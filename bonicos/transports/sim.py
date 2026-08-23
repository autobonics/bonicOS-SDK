"""``SimTransport`` — a fake robot with no hardware behind it.

Built on :class:`~bonicos.transports.mock.MockTransport` for the id counter,
ack handling, and telemetry dict — but where
``MockTransport`` is a *test double* (telemetry the test sets by hand, acks
the test scripts), this is a *physics stand-in*: ``drive`` actually
integrates a pose, ``servo_command`` actually ramps joints toward their
targets. ``robot.py`` cannot tell this transport from a real one, which is
the whole point — a fake-robot mode built on the real SDK instead of a
reimplementation of it in another language is what makes sim/robot parity
structural rather than maintained.

**Two hard constraints, not style preferences:**

1. :meth:`wait_for_update` must advance the simulation itself and return —
   one call, one tick. ``MockTransport.wait_for_update`` blocks on a
   ``threading.Event`` another thread sets; in a single-threaded host
   (a Pyodide Web Worker) nothing else runs to set it, so inheriting that
   behaviour would burn the whole timeout and always return ``False``.
   Integration is against wall-clock elapsed time, so the documented loop
   pattern (``while robot.wait_for_update(): ...``) behaves exactly like it
   does against a real robot, and a stray ``time.sleep(1)`` between calls is
   accounted for rather than silently lost.
2. No threads, no sockets, no ``js``/``pyodide.ffi`` imports. This module
   must import and run under Pyodide unchanged — pure Python, pure
   arithmetic.

Selection is the existing host-injection seam, nothing new::

    import bonicos
    from bonicos.transports.sim import SimTransport

    bonicos.use_transport(SimTransport())
    robot = bonicos.BonicBot()        # everything below is unchanged

or the ``BonicBot.simulated()`` shortcut (API.md §1) for the common case of
one script, one fake robot, nothing else registered.
"""

from __future__ import annotations

import base64
import heapq
import itertools
import math
import time
import zlib
from collections import deque
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple

from .. import protocol
from ..exceptions import CameraUnavailable
from .base import Frame
from .mock import MockTransport

#: A joint's animation state: (angle it started from, angle it's headed to,
#: the wall-clock time the move started, how long it should take) — all in
#: radians/seconds. Evaluated against elapsed time, never against a step
#: counter, for the same reason `drive` integrates against wall-clock time.
_Ramp = Tuple[float, float, float, float]


class _Obstacle(NamedTuple):
    """An obstacle footprint as an oriented bounding box (OBB).

    Half-extents are in the obstacle's *own* frame, so a rotated wall is
    described exactly rather than by a fattened world-aligned box. `theta`
    of 0 is the axis-aligned case and stays the common one — `cos_t`/`sin_t`
    are precomputed here because `_collides` runs per obstacle per tick and
    `_rasterize` runs per cell.

    The real robot navigates from a 2D laser scan, so a footprint is the
    whole of what an obstacle is here — height is a rendering concern the
    host keeps to itself (dev/SIMULATOR.md §4).
    """

    cx: float
    cy: float
    hx: float
    hy: float
    theta: float
    cos_t: float
    sin_t: float

    @classmethod
    def from_wire(cls, o: dict) -> "_Obstacle":
        """Build from the host's dict — ``x``/``y``/``sizeX``/``sizeY``
        (camelCase, world metres, FULL extents) plus an optional ``theta``
        in radians. The names are the frontend contract; see
        dev/SIMULATOR.md §6."""
        theta = float(o.get("theta", 0.0) or 0.0)
        return cls(
            cx=float(o["x"]),
            cy=float(o["y"]),
            hx=float(o["sizeX"]) / 2.0,
            hy=float(o["sizeY"]) / 2.0,
            theta=theta,
            cos_t=math.cos(theta),
            sin_t=math.sin(theta),
        )

    def to_local(self, x: float, y: float) -> Tuple[float, float]:
        """World point -> this obstacle's frame, where the box is the plain
        axis-aligned `[-hx, hx] x [-hy, hy]` and every test below is the
        simple one."""
        dx, dy = x - self.cx, y - self.cy
        return (
            dx * self.cos_t + dy * self.sin_t,
            -dx * self.sin_t + dy * self.cos_t,
        )

    def bounds(self, inflate: float = 0.0) -> Tuple[float, float, float, float]:
        """World-aligned AABB enclosing this (optionally inflated) OBB.

        Used for grid extent and to bound the rasterizer's scan. A rotated
        box reaches further in world x/y than its own half-extents, and
        forgetting that silently clips a diagonal wall at the grid edge.
        """
        hx, hy = self.hx + inflate, self.hy + inflate
        ex = hx * abs(self.cos_t) + hy * abs(self.sin_t)
        ey = hx * abs(self.sin_t) + hy * abs(self.cos_t)
        return (self.cx - ex, self.cy - ey, self.cx + ex, self.cy + ey)

    def contains(self, x: float, y: float, inflate: float = 0.0) -> bool:
        """Is the world point inside this OBB, grown by `inflate` on every
        side of its own frame?"""
        xr, yr = self.to_local(x, y)
        return abs(xr) <= self.hx + inflate and abs(yr) <= self.hy + inflate


#: Circular footprint used for collision (dev/SIMULATOR.md §3.1) and for inflating the
#: planner's occupancy grid so the planner can treat the robot as a point
#: (dev/SIMULATOR.md §3.4). The M1 base is roughly 0.35 m across; the margin keeps the
#: rendered mesh from visibly clipping obstacle geometry.
ROBOT_RADIUS = 0.22

# --- planning grid (dev/SIMULATOR.md §3.5) -----------------------------

GRID_RESOLUTION = 0.05  # m/cell
GRID_PADDING = 1.0  # m of free space around the content bounding box
#: Clearance added to the robot's radius when inflating the planner's grid.
#: `bonicAI-frontend/src/models/productCatalog.ts` duplicates this as
#: `PLANNER_INFLATION_MARGIN_M` — the world editor needs the number before a
#: run exists and cannot import Python. Change both.
INFLATION_MARGIN = 0.05

#: Default inflation, for the default radius. Per-instance inflation is
#: `self._inflation`; a robot is not obliged to be an M1.
INFLATION = ROBOT_RADIUS + INFLATION_MARGIN

#: Furthest a cell centre can sit from a shape that still covers part of
#: that cell. Used to keep rotated rasterization as conservative as the
#: axis-aligned span fill — see `SimTransport._cell_covered`.
_CELL_HALF_DIAGONAL = GRID_RESOLUTION * math.sqrt(2.0) / 2.0


def _floor_div(value: float) -> int:
    """World offset -> cell index. `math.floor`, not `int`: truncation goes
    toward zero and would fold the cells either side of an origin onto each
    other. Grid bounds are padded well past any obstacle so this is only
    ever fed positives today, but that is an invariant of the caller, not
    of the arithmetic."""
    return math.floor(value / GRID_RESOLUTION)


# --- Regulated Pure Pursuit (dev/SIMULATOR.md §3.4) — starting constants, tuned by watching
# --- the actual motion in the browser, not derived analytically. -----------

NAV_V_MAX = 0.35  # m/s
NAV_W_MAX = 1.20  # rad/s
NAV_A_MAX = 0.50  # m/s^2, used for goal-approach deceleration
LOOKAHEAD_MIN = 0.30  # m
LOOKAHEAD_MAX = 0.90  # m
LOOKAHEAD_GAIN = 1.50  # s  (lookahead = v * gain, clamped)
CURVATURE_GAIN = 0.60
GOAL_TOLERANCE = 0.10  # m
YAW_TOLERANCE = 0.10  # rad
ALIGN_THRESHOLD = 1.05  # rad (~60 deg) — rotate in place first

#: `_advance_navigation` states in which it actively drives the base, as
#: opposed to leaving `_linear_x`/`_angular_z` alone for manual `drive()`.
_NAV_ACTIVE = frozenset({"planning", "moving", "aligning"})


def _wrap_angle(theta: float) -> float:
    """Normalize to (-pi, pi] without a branchy modulo."""
    return math.atan2(math.sin(theta), math.cos(theta))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SimTransport(MockTransport):
    """A simulated BonicBot: differential-drive pose + ramped servos.

    Navigation is real (if simplified): `nav_goal` plans a path with A*,
    smooths it, and drives it with Regulated Pure Pursuit, arcing around
    ``obstacles`` the same way Nav2 does — see dev/SIMULATOR.md. Mapping
    and named locations are still stubs, exactly matching the real robot's
    own stub convention for the commands that are still stubs there. A
    little bookkeeping (which maps have been "saved", what nav mode is
    active) makes ``save_map``/``list_maps``/``enter_navigation_mode``/etc.
    behave sensibly for a demo without simulating SLAM.
    """

    #: Default `servo_command` duration if a caller's payload omits it
    #: (matches ``ArmController``'s SDK-side default of 1.0s).
    _DEFAULT_SERVO_DURATION_S = 1.0

    #: Plausible resting values — a freshly-constructed sim shouldn't look
    #: obviously fake at a glance (e.g. `0.0` battery reading as "dead").
    _BATTERY_VOLTAGE = 12.6
    _BATTERY_SOC = 100.0

    def __init__(
        self,
        *,
        joints: Optional[Sequence[str]] = None,
        obstacles: Optional[Sequence[dict]] = None,
        radius: float = ROBOT_RADIUS,
        cameras: Optional[Sequence[str]] = None,
    ) -> None:
        """A fully-capable simulated robot.

        ``joints`` simulates a robot built with fewer than the full 18
        actuators — servo count is a per-robot build option, so this is a real
        case worth being able to reproduce without hardware.

        ``obstacles`` are axis-aligned footprints (``{"x", "y", "sizeX",
        "sizeY", ...}``, world metres) fixed for this transport's lifetime —
        see dev/SIMULATOR.md §6. They are what `nav_goal` plans around
        and what manual `drive()` collides with.

        There is no ``model`` parameter: capability is not modelled anywhere in
        the SDK (PROTOCOL.md §3.1), so there is nothing for a model name to
        select. Mapping and named locations ack and do nothing here, matching
        the stub convention rather than a lite robot's hard failure.
        """
        super().__init__()
        fitted = list(joints) if joints is not None else list(protocol.JOINT_NAME_MAP)
        self._auth_result = {
            "robot_id": "SIM_001",
            "series": "SIM",
            "cameras": [],
        }

        #: Collision radius and the planner inflation derived from it.
        #: Host push (C3), because the three series are different sizes and
        #: a module constant can only ever be right for one of them.
        self._radius = float(radius)
        self._inflation = self._radius + INFLATION_MARGIN

        #: Camera names, ordered — `get_frame()` with no argument takes the
        #: first, so the order is contract, not convenience. These must be
        #: the *robot's* names (`bonicOS-robot-app/app/config.py` CAMERAS:
        #: "face", "docking") or a program that works here breaks there.
        self._cameras: List[str] = list(cameras) if cameras else ["main"]
        #: Cameras `start_camera()` has brought up. `read_frame` answers for
        #: these only; the preview panel is gated separately.
        self._live_cameras: set = set()

        #: Oriented footprints in world metres. Fixed for the lifetime of
        #: the transport — see SimRunMsg.obstacles.
        self._obstacles: List[_Obstacle] = [
            _Obstacle.from_wire(o) for o in (obstacles or [])
        ]
        #: snake_case URDF names of the fitted joints — what `joint_states`
        #: telemetry reports, and the set the servo ramps operate over.
        self._fitted_urdf = {
            protocol.JOINT_NAME_MAP[key]
            for key in fitted
            if key in protocol.JOINT_NAME_MAP
        }

        self._last_tick = time.monotonic()

        # --- base pose --------------------------------------------------
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._linear_x = 0.0
        self._angular_z = 0.0
        #: previous tick's linear velocity, for the finite-difference `ax`
        #: the synthesized IMU derives (dev/SIMULATOR.md §3.3).
        self._prev_linear_x = 0.0

        # --- planning grid (dev/SIMULATOR.md §3.5). Obstacles are fixed for the
        # --- transport's lifetime, so this is built once — grown lazily in
        # --- `_maybe_grow_grid` only if a later goal/pose falls outside it.
        self._grid_extra_points: List[Tuple[float, float]] = [(self._x, self._y)]
        self._build_grids()

        # --- navigation state machine (dev/SIMULATOR.md §3.4): idle -> planning -> moving
        # --- -> aligning -> succeeded | failed | canceled.
        self._nav_state = "idle"
        self._nav_path: List[Tuple[float, float]] = []
        self._nav_path_index = 0
        self._nav_goal_theta: Optional[float] = None
        self._nav_aligned = False  # latches the initial rotate-in-place
        self._nav_goal_id = 0

        # --- servos: every registered joint starts at 0 rad, like a robot
        # --- freshly powered on. Keyed by snake_case URDF name — what a
        # --- real /joint_states reports (protocol.JOINT_NAME_MAP).
        self._joint_positions: Dict[str, float] = {
            name: 0.0 for name in sorted(self._fitted_urdf)
        }
        self._ramps: Dict[str, _Ramp] = {}

        # --- nav/mapping bookkeeping (no Nav2 — just tracking what was
        # --- asked for, so list_maps()/get_nav_mode()/etc. aren't lies).
        self._maps: List[str] = []
        self._nav_mode = "idle"
        self._nav_map: Optional[str] = None

        # --- camera (dev/SIMULATOR.md §3.6): no provider until the host installs one via
        # --- `set_frame_provider` — a native user never does, so this stays
        # --- exactly like the pre-Phase-2 stub (`supports_camera = False`,
        # --- `start_camera` raises) unless a browser host is behind it.
        #: ``(camera, x, y, theta, neck_yaw, neck_pitch) -> (bgr, w, h) | None``
        self._frame_provider: Optional[
            Callable[[str, float, float, float, float, float], Optional[tuple]]
        ] = None
        self._camera_live = False
        #: Host push (C3): whether bonicAI-frontend/SimulatorWorldAndCameraOverview.md §5's preview panel is open. Gates
        #: `_maybe_render_camera` together with `_camera_live` — see there.
        self._camera_preview_open = False
        self._frames: Dict[str, Tuple[bytes, int, int]] = {}

        # --- speech (dev/SIMULATOR.md §3.7): no provider until the host installs
        # --- one via `set_speech_provider` — same seam as the camera's
        # --- `set_frame_provider`. No provider means `speak()` keeps the
        # --- pre-existing silent-ack stub behaviour.
        #: ``(text, voice) -> bool | None``
        self._speech_provider: Optional[Callable[[str, Optional[str]], Optional[bool]]] = None

        self._publish_pose_and_odom()
        self._publish_joint_states()
        self.set_telemetry(
            protocol.EVENT_BATTERY,
            {
                "voltage": self._BATTERY_VOLTAGE,
                "current": 0.0,
                "soc": self._BATTERY_SOC,
            },
        )

    # --- camera (Phase 2 / dev/SIMULATOR.md §3.6) -----------------------------------------
    # `sim.py` never imports `js` (C2): the browser host installs a plain
    # callable via `set_frame_provider` — the same seam as `jointBoundsDeg` —
    # and this module only ever calls it, never reaches for it. A native user
    # never installs one, so `supports_camera` stays False and the API fails
    # with a clear message, exactly like the pre-Phase-2 stub did.

    supports_camera = False

    def set_frame_provider(
        self,
        provider: Optional[
            Callable[[str, float, float, float, float, float], Optional[tuple]]
        ],
    ) -> None:
        """Install a host callable returning ``(bgr_bytes, width, height)``
        for a camera name at a given pose and neck angle, or ``None`` if no
        frame is available yet. Called as
        ``provider(camera, x, y, theta, neck_yaw, neck_pitch)``.

        Host-side glue, like the render-pump hooks in ``simWorker.ts``:
        ``sim.py`` stores and calls a plain callable and never imports
        ``js``, so this module still imports and runs unchanged outside a
        browser (C2). A native user simply never installs one and the
        camera stays unavailable, exactly as before this task.

        The provider is invoked from ``_tick()``, not from ``read_frame()``
        (dev/SIMULATOR.md §3.6's amendment: rendering rides the same
        pump hook points as pose/telemetry — ``wait_for_update()`` and
        ``read_telemetry()`` — so the bonicAI-frontend/SimulatorWorldAndCameraOverview.md §5 preview panel stays live even
        for a program that never calls ``read_frame()``, and so the render
        rate is capped at the pump rate no matter how often Python asks).
        See ``_maybe_render_camera`` for the gating.

        The plan's own sketch of this seam has the provider take just a
        camera name and read pose back out through ``get_state()``. Wired up
        literally, that recurses: ``get_state()`` calls ``_tick()``, which
        calls this method, which would call the provider, which would call
        ``get_state()`` again. Passing ``x``/``y``/``theta``/neck angles as
        plain floats the provider receives is simpler than it looks and
        sidesteps the problem entirely — ``_tick()`` already has them on
        hand with no further method call, so there's nothing to pull back
        out.

        ``neck_yaw``/``neck_pitch`` are ``_joint_positions["neck_yaw_joint"]``
        / ``["neck_pitch_joint"]`` (radians, ramped by ``_advance_servos`` —
        see ``set_neck``/``look_left``/``look_right``/``set_single_servo``,
        all of which route through ``CMD_SERVO_COMMAND`` and genuinely move
        these). ``HeadController.look(pan, tilt)`` is a **different, and
        deliberately inert, command** — ``CMD_HEAD_LOOK`` is a v1 stub on the
        real robot too (``controllers/head.py``'s module docstring), so it
        acks and moves nothing here. If a later change makes this camera
        respond to ``head.look()`` instead of (or in addition to) the neck
        joints, that is a parity regression: the simulator would pan while a
        real robot sits still.
        """
        self._frame_provider = provider
        self.supports_camera = provider is not None
        self._auth_result["cameras"] = list(self._cameras) if provider else []

    def set_camera_preview_open(self, is_open: bool) -> None:
        """Host push (C3): whether bonicAI-frontend/SimulatorWorldAndCameraOverview.md §5's preview panel is open.

        Combines with ``_camera_live`` (set by ``start_camera``/
        ``stop_camera``) to decide whether ``_tick()`` renders a frame each
        pump tick — neither implies the other. Preview open with no
        ``start_camera()`` renders and displays, but ``read_frame()`` still
        returns ``None`` (the API contract, enforced the same way a real
        robot enforces it). ``start_camera()`` with the preview closed keeps
        ``read_frame()`` live with nothing displayed.
        """
        self._camera_preview_open = bool(is_open)

    # --- speech (dev/SIMULATOR.md §3.7) -----------------------------------
    # Same seam as the camera's `set_frame_provider` (C2: no `js` import
    # here) — a browser host wires this to Web Speech `speechSynthesis`, a
    # native host to whatever local TTS it has, and this module only ever
    # calls a plain callable it was handed.

    def set_speech_provider(
        self, provider: Optional[Callable[[str, Optional[str]], Optional[bool]]]
    ) -> None:
        """Install a host callable invoked as ``provider(text, voice)`` for
        every ``speak()`` call, ``voice`` being ``None`` when the caller
        didn't pass one.

        Unlike the camera, there is no hard-failure path: `SystemController
        .speak()` (PROTOCOL.md §5.6) already acks unconditionally on a real
        robot with no TTS route wired up yet (`command_handlers.py`'s own
        `speak` is a stub), so a native user who never installs a provider
        keeps getting that same silent, successful ack — installing a
        provider is purely additive, never a new way for `speak()` to fail
        that a program written before this seam existed didn't already
        handle.

        The provider's return value maps to the command's `ok`: `True`/
        `False` pass through, and `None` (a fire-and-forget bridge with
        nothing to report, e.g. a bare `speechSynthesis.speak()` call) is
        treated as success — the same "no news is good news" default the
        pre-existing stub always gave.
        """
        self._speech_provider = provider

    def start_camera(self, cameras: list) -> None:
        if self._frame_provider is None:
            raise CameraUnavailable(
                "the simulator has no camera — the browser host installs a "
                "frame provider automatically; off-browser, there is "
                "nothing for this transport to render from"
            )
        # Track *which* cameras, not merely that some camera is up: a
        # multi-camera robot answers `read_frame("docking")` only if docking
        # was actually started, exactly as the real one does.
        self._live_cameras = set(cameras) if cameras else set(self._cameras)
        self._camera_live = True

    def stop_camera(self) -> None:
        self._live_cameras = set()
        self._camera_live = False

    def read_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        name = camera or self._cameras[0]
        if name not in self._live_cameras:
            # Not started (or not a camera this robot has). The preview panel
            # may well be rendering it — that deliberately does not make it
            # readable, since a real robot requires `start_camera()` first.
            return None
        entry = self._frames.get(name)
        if entry is None:
            return None
        # Lazy, mirroring transports/_camera_link.py:60-68 — base.py:16's
        # `Frame = Any` exists specifically so numpy stays off the base
        # import path and a driving-only install never pays for it.
        import numpy as np

        bgr_bytes, width, height = entry
        return np.frombuffer(bgr_bytes, dtype=np.uint8).reshape(height, width, 3).copy()

    def _maybe_render_camera(self) -> None:
        """The render pump's camera hook, called from `_tick()` — see
        `set_frame_provider`'s docstring for why this lives here and not in
        `read_frame()`. Gated so idle GPU work isn't wasted when nobody
        wants a frame (`set_camera_preview_open`)."""
        if self._frame_provider is None:
            return
        # Render only what someone actually wants: the started cameras, plus
        # the first one if the preview panel is open. Rendering every camera
        # on an M1 every tick would double the GPU cost of a student who only
        # ever asked for the face — the same waste the gate above exists to
        # avoid.
        wanted = set(self._live_cameras)
        if self._camera_preview_open:
            wanted.add(self._cameras[0])
        if not wanted:
            return
        # Neck angles the provider aims the camera with. Read from
        # `_joint_positions` (already ramped by `_advance_servos`) rather
        # than `CMD_HEAD_LOOK` — see `set_frame_provider`'s docstring for
        # why the two must never be conflated. `.get(..., 0.0)` covers a
        # robot built without the neck fitted (`joints=` excludes it).
        #
        # They are passed for every camera; whether a given one *turns* with
        # the neck is the host's business, since only the host knows the
        # mount. A chassis-mounted docking camera must ignore them.
        neck_yaw = self._joint_positions.get("neck_yaw_joint", 0.0)
        neck_pitch = self._joint_positions.get("neck_pitch_joint", 0.0)
        for name in self._cameras:  # stable order, not set order
            if name not in wanted:
                continue
            result = self._frame_provider(
                name, self._x, self._y, self._theta, neck_yaw, neck_pitch
            )
            if result is None:
                continue
            bgr_bytes, width, height = result
            self._frames[name] = (bytes(bgr_bytes), int(width), int(height))

    # --- Transport protocol ----------------------------------------------

    def send(self, msg: dict) -> int:
        self._tick()  # settle everything up to "now" before this arrives
        cmd_type = msg.get("type")

        if cmd_type == protocol.CMD_DRIVE:
            # Manual drive is teleop override and must win over an active
            # goal, exactly as it does on a real robot (dev/SIMULATOR.md §3.4).
            if self._nav_state in _NAV_ACTIVE:
                self._nav_state = "canceled"
                self._nav_path = []
                self._publish_nav_status("canceled", 0.0)
                self._publish_plan([])
            self._linear_x = float(msg.get("linear_x", 0.0))
            self._angular_z = float(msg.get("angular_z", 0.0))
            return super().send(msg)  # unacked (protocol.UNACKED_COMMANDS)

        cmd_id = super().send(msg)
        if cmd_type == protocol.CMD_SERVO_COMMAND:
            result = self._start_servo_command(msg)
        elif cmd_type == protocol.CMD_NAV_GOAL:
            result = self._start_goal(
                float(msg.get("x", 0.0)),
                float(msg.get("y", 0.0)),
                float(msg.get("theta", 0.0)),
            )
        elif cmd_type == protocol.CMD_NAVIGATE_THROUGH_WAYPOINTS:
            result = self._start_waypoints(msg.get("waypoints", []))
        elif cmd_type == protocol.CMD_CANCEL_NAV:
            result = self._cancel_navigation()
        else:
            result = self._build_ack(msg)
        self.script_ack_for_id(cmd_id, result)
        return cmd_id

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        """One call, one tick — see the module docstring's constraint 1."""
        self._tick()
        return True

    def read_telemetry(self) -> dict:
        self._tick()
        return super().read_telemetry()

    def close(self) -> None:
        self._linear_x = 0.0
        self._angular_z = 0.0
        super().close()

    # --- simulator-only surface ------------------------------------------

    def get_state(self) -> dict:
        """Pose + joint angles, for an embedding runtime to render.

        Deliberately just a plain method returning a plain dict — no
        ``postMessage``, no ``js`` import, nothing browser-specific. A
        native user running a fake robot should not pay for browser
        machinery; the host decides how (or whether) to ship this out.
        """
        self._tick()
        return {
            "pose": {"x": self._x, "y": self._y, "theta": self._theta},
            "joints": dict(self._joint_positions),
        }

    # --- physics -----------------------------------------------------------

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if dt > 0:
            self._advance_navigation(dt)  # sets _linear_x/_angular_z
            self._integrate_drive(dt)
        self._advance_servos(now)
        self._publish_pose_and_odom()
        self._publish_joint_states()
        self._publish_imu(dt)
        self._maybe_render_camera()

    def _integrate_drive(self, dt: float) -> None:
        if self._linear_x == 0.0 and self._angular_z == 0.0:
            return
        nx = self._x + self._linear_x * math.cos(self._theta) * dt
        ny = self._y + self._linear_x * math.sin(self._theta) * dt
        if self._collides(nx, ny):
            # Blocked: the base stops translating but may still rotate in
            # place, which is what a real robot pinned against an obstacle
            # does and is what lets a stuck program turn away.
            self._linear_x = 0.0
        else:
            self._x, self._y = nx, ny
        self._theta = _wrap_angle(self._theta + self._angular_z * dt)

    def _collides(self, x: float, y: float) -> bool:
        """Circle-vs-OBB against the RAW footprints.

        Deliberately not the planner's inflated grid: that grid is a
        planning abstraction at a fixed resolution, this is the physical
        check, and a robot legitimately drives closer to a wall under
        manual `drive()` than the planner would ever route it.

        Rotation costs one 2x2 transform per obstacle: in the box's own
        frame the test is the same clamp-and-compare it always was, and it
        stays exact rather than falling back to the enclosing AABB.
        """
        for obs in self._obstacles:
            xr, yr = obs.to_local(x, y)
            nx = min(max(xr, -obs.hx), obs.hx)
            ny = min(max(yr, -obs.hy), obs.hy)
            if (xr - nx) ** 2 + (yr - ny) ** 2 < self._radius**2:
                return True
        return False

    def _publish_imu(self, dt: float) -> None:
        """IMU derived from the motion this transport already integrates.

        Not an independent sensor model: `gz` *is* the commanded yaw rate,
        `ay` *is* the centripetal term of the arc being driven, and `ax` is
        the finite difference of `vx`. Deriving it from the pose integration
        is what keeps it consistent with `odom` under every manoeuvre — a
        standalone noise model would drift out of agreement with the pose
        the same tick publishes.
        """
        v, w = self._linear_x, self._angular_z
        ax = (v - self._prev_linear_x) / dt if dt > 0 else 0.0
        self._prev_linear_x = v
        self.set_telemetry(
            protocol.EVENT_IMU,
            {
                "ax": ax,  # m/s^2, forward
                "ay": v * w,  # m/s^2, centripetal on the current arc
                "az": 9.81,  # m/s^2, gravity — the base stays level
                "gx": 0.0,  # rad/s — a wheeled base does not roll
                "gy": 0.0,  # rad/s — or pitch
                "gz": w,  # rad/s, yaw rate
            },
        )

    def _advance_servos(self, now: float) -> None:
        finished = []
        for name, (start_pos, target, start_time, duration) in self._ramps.items():
            frac = (now - start_time) / duration
            if frac >= 1.0:
                self._joint_positions[name] = target
                finished.append(name)
            else:
                self._joint_positions[name] = start_pos + (target - start_pos) * max(
                    0.0, frac
                )
        for name in finished:
            del self._ramps[name]

    def _start_servo_command(self, msg: dict) -> dict:
        """Begin ramping every named joint toward its target.

        Preemption matches real hardware (verified live): a joint already
        mid-ramp restarts from its *current
        interpolated position*, not from its old start or target — no jerk,
        no queueing. Unrecognized keys (typo'd joint names) are reported
        ``unknown`` and never touch ``_joint_positions``, same as the
        server excluding them from ``servo_command``'s ack.

        A joint that is *valid but not fitted* on this simulated robot is
        reported ``unknown`` too. That is what a real server must do — a
        server that accepted it instead would leave ``set_servos(wait=True)``
        blocking until timeout on an actuator that will never move, since the
        SDK only excludes ``unknown`` keys before waiting for convergence.
        """
        servos = msg.get("servos", {})
        duration = float(msg.get("duration") or self._DEFAULT_SERVO_DURATION_S)
        duration = max(duration, 1e-6)  # guard div-by-zero on a 0.0 duration
        now = time.monotonic()

        unknown = []
        groups = set()
        for camel_key, target_rad in servos.items():
            snake_name = protocol.JOINT_NAME_MAP.get(camel_key)
            if snake_name is None or snake_name not in self._fitted_urdf:
                unknown.append(camel_key)
                continue
            groups.add(protocol.JOINT_GROUP_OF.get(camel_key, camel_key))
            start_pos = self._joint_positions.get(snake_name, 0.0)
            self._ramps[snake_name] = (start_pos, float(target_rad), now, duration)

        return {"ok": True, "groups": sorted(groups), "unknown": unknown}

    def _publish_pose_and_odom(self) -> None:
        self.set_telemetry(
            protocol.EVENT_POSE, {"x": self._x, "y": self._y, "theta": self._theta}
        )
        # Precise-motion polls `odom`, not `pose` —
        # both are published from the same integrated pose here since this
        # transport has no separate localizer/odometry source to diverge.
        self.set_telemetry(
            protocol.EVENT_ODOM,
            {
                "x": self._x,
                "y": self._y,
                "theta": self._theta,
                "vx": self._linear_x,
                "vtheta": self._angular_z,
            },
        )

    def _publish_joint_states(self) -> None:
        names = list(self._joint_positions.keys())
        self.set_telemetry(
            protocol.EVENT_JOINT_STATES,
            {"name": names, "position": [self._joint_positions[n] for n in names]},
        )

    # --- planning grid (dev/SIMULATOR.md §3.5) --------------------------------------------

    def _build_grids(self) -> None:
        """(Re)build `_grid_raw`/`_grid_inflated` from obstacles + seed points.

        Bounds are the bounding box of every obstacle and every point in
        `_grid_extra_points` (the robot's start pose, plus any later goal
        that fell outside a previous grid — see `_maybe_grow_grid`), padded
        by `GRID_PADDING`. Called once from `__init__` and again, rarely,
        when a goal needs more room than the grid already covers.
        """
        xs = [p[0] for p in self._grid_extra_points]
        ys = [p[1] for p in self._grid_extra_points]
        for obs in self._obstacles:
            # The OBB's world-aligned bounds, not its own half-extents: a
            # rotated wall reaches further in world x/y than `hx`/`hy`, and
            # sizing the grid to the latter clips it at the edge.
            min_x, min_y, max_x, max_y = obs.bounds()
            xs += [min_x, max_x]
            ys += [min_y, max_y]
        min_x, max_x = min(xs) - GRID_PADDING, max(xs) + GRID_PADDING
        min_y, max_y = min(ys) - GRID_PADDING, max(ys) + GRID_PADDING

        self._grid_ox = min_x
        self._grid_oy = min_y
        self._grid_w = max(1, int(math.ceil((max_x - min_x) / GRID_RESOLUTION)))
        self._grid_h = max(1, int(math.ceil((max_y - min_y) / GRID_RESOLUTION)))
        self._grid_raw = self._rasterize(0.0)
        self._grid_inflated = self._rasterize(self._inflation)
        self._publish_grids()

    def _publish_grids(self) -> None:
        """Publish `_grid_raw`/`_grid_inflated` as `EVENT_MAP`/`EVENT_COSTMAP`
        (dev/SIMULATOR.md §5). Called from `_build_grids`, so it fires once from
        `__init__` and again on the rare later regrow — never per tick, since
        between those two points the grids don't change.
        """
        info = {
            "resolution": GRID_RESOLUTION,
            "width": self._grid_w,
            "height": self._grid_h,
            "origin": {"x": self._grid_ox, "y": self._grid_oy, "theta": 0.0},
        }
        for event, grid in (
            (protocol.EVENT_MAP, self._grid_raw),
            (protocol.EVENT_COSTMAP, self._grid_inflated),
        ):
            self.set_telemetry(
                event,
                {
                    "info": info,
                    "data_b64": base64.b64encode(zlib.compress(bytes(grid))).decode(
                        "ascii"
                    ),
                },
            )

    def _rasterize(self, inflate: float) -> bytearray:
        """Grow each obstacle by `inflate` on every side of its own frame,
        then fill the cells it covers.

        Axis-aligned boxes take the exact path: grow the rectangle, fill the
        span. O(n) rather than O(cells x kernel), exact on the faces, only
        slightly conservative at the corners (square instead of rounded),
        which errs safe.

        A rotated box has no such span, so its cells are tested individually
        over its world-aligned bounds. Two things make that safe rather than
        merely plausible — see `_cell_covered`.
        """
        grid = bytearray(self._grid_w * self._grid_h)
        for obs in self._obstacles:
            min_x, min_y, max_x, max_y = obs.bounds(inflate)
            gx0 = max(0, _floor_div(min_x - self._grid_ox))
            gy0 = max(0, _floor_div(min_y - self._grid_oy))
            gx1 = min(self._grid_w - 1, _floor_div(max_x - self._grid_ox))
            gy1 = min(self._grid_h - 1, _floor_div(max_y - self._grid_oy))
            axis_aligned = obs.sin_t == 0.0 and obs.cos_t > 0.0
            for row in range(gy0, gy1 + 1):
                base = row * self._grid_w
                for col in range(gx0, gx1 + 1):
                    if axis_aligned or self._cell_covered(obs, row, col, inflate):
                        grid[base + col] = 100
        return grid

    def _cell_covered(self, obs: _Obstacle, row: int, col: int, inflate: float) -> bool:
        """Does the cell at (row, col) overlap `obs` grown by `inflate`?

        Tested as "is the cell *centre* inside the box, grown by a further
        half cell diagonal" — which is what keeps the rotated path from
        being weaker than the axis-aligned one it sits beside.

        That path marks every cell the rectangle *touches*, however
        slightly, because it floors both edges of the span. Testing bare
        centres would instead mark only cells whose middle is covered, so a
        wall thinner than a cell laid on the diagonal could thread between
        centres and rasterize to a dotted line — through which A* would
        happily plan a path straight into it. Padding by the half diagonal
        (the furthest a covered cell's centre can sit from the box) closes
        that, and errs the same direction as the corner conservatism above:
        an obstacle is never smaller than it really is.
        """
        x, y = self._cell_to_world(row, col)
        return obs.contains(x, y, inflate + _CELL_HALF_DIAGONAL)

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = _floor_div(x - self._grid_ox)
        row = _floor_div(y - self._grid_oy)
        return row, col

    def _cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self._grid_ox + (col + 0.5) * GRID_RESOLUTION
        y = self._grid_oy + (row + 0.5) * GRID_RESOLUTION
        return x, y

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self._grid_h and 0 <= col < self._grid_w

    def _occupied(self, grid: bytearray, row: int, col: int) -> bool:
        if not self._in_bounds(row, col):
            return True
        return grid[row * self._grid_w + col] != 0

    def _maybe_grow_grid(self, *points: Tuple[float, float]) -> None:
        grown = False
        for x, y in points:
            if not self._in_bounds(*self._world_to_cell(x, y)):
                self._grid_extra_points.append((x, y))
                grown = True
        if grown:
            self._build_grids()

    # --- A* (dev/SIMULATOR.md §3.4) --------------------------------------------------------

    def _nearest_free_cell(self, start: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """BFS outward for the nearest cell not occupied in `_grid_inflated`.

        Handles the robot spawning (or being driven) into what the planner
        treats as occupied — e.g. right next to a box.
        """
        if not self._in_bounds(*start):
            return None
        if not self._occupied(self._grid_inflated, *start):
            return start
        seen = {start}
        queue = deque([start])
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if not self._in_bounds(nr, nc) or (nr, nc) in seen:
                    continue
                seen.add((nr, nc))
                if not self._occupied(self._grid_inflated, nr, nc):
                    return (nr, nc)
                queue.append((nr, nc))
        return None

    def _astar(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> Optional[List[Tuple[int, int]]]:
        """8-connected A* over `_grid_inflated`, corner-cutting forbidden."""
        free_start = self._nearest_free_cell(start)
        if free_start is None or self._occupied(self._grid_inflated, *goal):
            return None
        start = free_start
        if start == goal:
            return [start]

        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        )
        counter = itertools.count()
        open_heap = [(heuristic(start, goal), next(counter), start)]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            if current == goal:
                path = [current]
                while current != start:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            r, c = current
            for dr, dc, cost in neighbors:
                nr, nc = r + dr, c + dc
                if self._occupied(self._grid_inflated, nr, nc):
                    continue
                if dr and dc:
                    # Forbid diagonal corner-cutting: both orthogonal
                    # neighbours of the diagonal move must be free too.
                    if self._occupied(self._grid_inflated, r + dr, c) or self._occupied(
                        self._grid_inflated, r, c + dc
                    ):
                        continue
                neighbor = (nr, nc)
                if neighbor in closed:
                    continue
                tentative = g_score[current] + cost
                if tentative < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = tentative
                    came_from[neighbor] = current
                    heapq.heappush(
                        open_heap,
                        (
                            tentative + heuristic(neighbor, goal),
                            next(counter),
                            neighbor,
                        ),
                    )
        return None

    # --- path smoothing (dev/SIMULATOR.md §3.4, 5d) ----------------------------------------

    def _line_of_sight(self, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        """True if the straight segment between cells `a` and `b` never
        crosses an occupied `_grid_inflated` cell. Sampled in world space at
        half the grid resolution rather than a raw Bresenham walk, so a
        near-diagonal segment can't slip between two occupied cells at a
        shared corner."""
        ax, ay = self._cell_to_world(*a)
        bx, by = self._cell_to_world(*b)
        dist = math.hypot(bx - ax, by - ay)
        steps = max(1, int(dist / (GRID_RESOLUTION * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            x, y = ax + (bx - ax) * t, ay + (by - ay) * t
            if self._occupied(self._grid_inflated, *self._world_to_cell(x, y)):
                return False
        return True

    def _shortcut(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """String-pulling: from each kept point, jump to the furthest later
        point still in clear line of sight. Turns 8-connected staircases
        into the straight diagonals a human would draw."""
        if len(path) < 3:
            return path
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._line_of_sight(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    @staticmethod
    def _chaikin(
        pts: List[Tuple[float, float]], iterations: int = 2
    ) -> List[Tuple[float, float]]:
        for _ in range(iterations):
            if len(pts) < 3:
                break
            out = [pts[0]]
            for a, b in zip(pts, pts[1:]):
                out.append((a[0] * 0.75 + b[0] * 0.25, a[1] * 0.75 + b[1] * 0.25))
                out.append((a[0] * 0.25 + b[0] * 0.75, a[1] * 0.25 + b[1] * 0.75))
            out.append(pts[-1])
            pts = out
        return pts

    def _smooth_and_guard(
        self, shortcut_world: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """Chaikin cuts corners inward, so a smoothed point can in principle
        clip an inflated cell. Drop any that do, falling back to the nearest
        pre-Chaikin point — in practice INFLATION's margin absorbs it, but
        the check is cheap and this is the regression guard against a path
        that visibly grazes a box."""
        smoothed = self._chaikin(shortcut_world)
        guarded = []
        for pt in smoothed:
            if self._occupied(self._grid_inflated, *self._world_to_cell(*pt)):
                nearest = min(
                    shortcut_world,
                    key=lambda p: (p[0] - pt[0]) ** 2 + (p[1] - pt[1]) ** 2,
                )
                guarded.append(nearest)
            else:
                guarded.append(pt)
        return guarded

    def _plan_path(
        self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        self._maybe_grow_grid(start_xy, goal_xy)
        goal_rc = self._world_to_cell(*goal_xy)
        if not self._in_bounds(*goal_rc):
            return None
        cell_path = self._astar(self._world_to_cell(*start_xy), goal_rc)
        if cell_path is None:
            return None
        shortcut_cells = self._shortcut(cell_path)
        world_path = [self._cell_to_world(r, c) for r, c in shortcut_cells]
        world_path[0] = start_xy
        world_path[-1] = goal_xy
        guarded = self._smooth_and_guard(world_path)
        guarded[0] = start_xy
        guarded[-1] = goal_xy
        return guarded

    # --- Regulated Pure Pursuit (dev/SIMULATOR.md §3.4) ------------------------------------

    def _advance_navigation(self, dt: float) -> None:
        if self._nav_state == "moving":
            self._pursue_path(dt)
        elif self._nav_state == "aligning":
            self._align_to_goal_theta(dt)
        # idle/terminal: leave _linear_x/_angular_z alone for manual drive().

    def _pursue_path(self, dt: float) -> None:
        path = self._nav_path
        goal_x, goal_y = path[-1]
        dist_to_goal = math.hypot(goal_x - self._x, goal_y - self._y)

        # 1. Advance a monotonic index to the closest point ahead, so the
        # robot can't latch onto an earlier segment after crossing its path.
        idx = self._nav_path_index
        best_idx, best_d = idx, math.hypot(
            path[idx][0] - self._x, path[idx][1] - self._y
        )
        for j in range(idx, len(path)):
            d = math.hypot(path[j][0] - self._x, path[j][1] - self._y)
            if d < best_d:
                best_idx, best_d = j, d
        self._nav_path_index = best_idx

        # 2. Lookahead point, walked forward along the path by arclength.
        lookahead = _clamp(
            self._linear_x * LOOKAHEAD_GAIN, LOOKAHEAD_MIN, LOOKAHEAD_MAX
        )
        px, py = self._lookahead_point(best_idx, lookahead)

        # 3. Into the robot frame.
        dx, dy = px - self._x, py - self._y
        xr = dx * math.cos(self._theta) + dy * math.sin(self._theta)
        yr = -dx * math.sin(self._theta) + dy * math.cos(self._theta)

        # 4. Initial alignment only — latched so it can never re-fire mid
        # path (that re-entry IS the stop-and-turn stutter this design
        # exists to avoid).
        if not self._nav_aligned:
            bearing = math.atan2(yr, xr)
            if abs(bearing) > ALIGN_THRESHOLD:
                self._linear_x = 0.0
                self._angular_z = math.copysign(NAV_W_MAX, yr)
                self._publish_nav_status("moving", dist_to_goal)
                return
            self._nav_aligned = True

        # 5. Curvature and velocity.
        dist_sq = xr * xr + yr * yr
        curvature = (2.0 * yr / dist_sq) if dist_sq > 1e-6 else 0.0

        v = NAV_V_MAX
        v = min(v, NAV_V_MAX / (1.0 + abs(curvature) * CURVATURE_GAIN))
        v = min(v, math.sqrt(2.0 * NAV_A_MAX * max(dist_to_goal, 0.0)))
        v = max(v, 0.05) if dist_to_goal > GOAL_TOLERANCE else 0.0

        w = _clamp(v * curvature, -NAV_W_MAX, NAV_W_MAX)

        self._linear_x, self._angular_z = v, w

        if dist_to_goal <= GOAL_TOLERANCE:
            # Arrived at the goal position — hand off to the final rotate
            # immediately, so this tick's telemetry already reads
            # "aligning" rather than "moving" with a zero velocity that
            # would look like the mid-path stutter this design avoids.
            self._nav_state = "aligning"
            self._publish_nav_status("aligning", dist_to_goal)
        else:
            self._publish_nav_status("moving", dist_to_goal)

    def _lookahead_point(self, start_idx: int, lookahead: float) -> Tuple[float, float]:
        path = self._nav_path
        if start_idx >= len(path) - 1:
            return path[-1]
        remaining = lookahead
        prev = path[start_idx]
        for i in range(start_idx + 1, len(path)):
            cur = path[i]
            seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            if seg >= remaining:
                t = remaining / seg if seg > 0 else 0.0
                return (
                    prev[0] + (cur[0] - prev[0]) * t,
                    prev[1] + (cur[1] - prev[1]) * t,
                )
            remaining -= seg
            prev = cur
        return path[-1]

    def _align_to_goal_theta(self, dt: float) -> None:
        target = self._nav_goal_theta if self._nav_goal_theta is not None else 0.0
        diff = _wrap_angle(target - self._theta)
        if abs(diff) <= YAW_TOLERANCE:
            self._finish_goal("succeeded")
            return
        self._linear_x = 0.0
        self._angular_z = _clamp(diff / max(dt, 1e-3), -NAV_W_MAX, NAV_W_MAX)
        self._publish_nav_status("aligning", 0.0)

    def _finish_goal(self, status: str) -> None:
        self._nav_state = status
        self._nav_path = []
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._publish_nav_status(status, 0.0)
        self._publish_plan([])

    def _publish_nav_status(self, status: str, distance_to_goal: float) -> None:
        self.set_telemetry(
            protocol.EVENT_NAV_STATUS,
            {"status": status, "distance_to_goal": distance_to_goal},
        )

    def _publish_plan(self, path: List[Tuple[float, float]]) -> None:
        """`EVENT_PLAN` (dev/SIMULATOR.md §5): the smoothed path while a goal is active,
        `[]` once idle/terminal. Called once per plan change — a new goal, or
        a transition to succeeded/failed/canceled — not per tick."""
        self.set_telemetry(
            protocol.EVENT_PLAN, {"points": [[p[0], p[1]] for p in path]}
        )

    # --- navigation commands (dev/SIMULATOR.md §3.4) ---------------------------------------

    def _start_goal(self, x: float, y: float, theta: float) -> dict:
        path = self._plan_path((self._x, self._y), (x, y))
        if path is None:
            self._nav_state = "failed"
            self._nav_path = []
            self._publish_nav_status("failed", 0.0)
            self._publish_plan([])
            return {"ok": False}
        self._nav_goal_id += 1
        self._nav_path = path
        self._nav_path_index = 0
        self._nav_aligned = False
        self._nav_goal_theta = theta
        self._nav_state = "moving"
        dist = math.hypot(x - self._x, y - self._y)
        self._publish_nav_status("moving", dist)
        self._publish_plan(path)
        return {"ok": True, "goal_id": str(self._nav_goal_id)}

    def _start_waypoints(self, waypoints: List[dict]) -> dict:
        if not waypoints:
            self._nav_state = "failed"
            self._nav_path = []
            self._publish_nav_status("failed", 0.0)
            self._publish_plan([])
            return {"ok": False}

        full_path: List[Tuple[float, float]] = []
        cur = (self._x, self._y)
        for wp in waypoints:
            leg_goal = (float(wp["x"]), float(wp["y"]))
            leg = self._plan_path(cur, leg_goal)
            if leg is None:
                self._nav_state = "failed"
                self._nav_path = []
                self._publish_nav_status("failed", 0.0)
                self._publish_plan([])
                return {"ok": False}
            # Concatenate into one continuous path — do not stop at
            # intermediate waypoints (drop the duplicate junction point).
            full_path.extend(leg[1:] if full_path else leg)
            cur = leg_goal

        self._nav_goal_id += 1
        self._nav_path = full_path
        self._nav_path_index = 0
        self._nav_aligned = False
        self._nav_goal_theta = float(waypoints[-1].get("theta", 0.0))
        self._nav_state = "moving"
        dist = math.hypot(cur[0] - self._x, cur[1] - self._y)
        self._publish_nav_status("moving", dist)
        self._publish_plan(full_path)
        return {"ok": True, "goal_id": str(self._nav_goal_id)}

    def _cancel_navigation(self) -> dict:
        self._nav_state = "canceled"
        self._nav_path = []
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._publish_nav_status("canceled", 0.0)
        self._publish_plan([])
        return {"canceled": True}

    # --- ack bookkeeping for everything that isn't drive/servo_command/nav -

    def _build_ack(self, msg: dict) -> dict:
        """Success-shaped acks for the rest of the command surface.

        Mapping and named-location commands acknowledge and otherwise do
        nothing, matching the real robot's own stub convention for the
        commands that are still stubs there — navigation is NOT a stub here
        (see `_start_goal`/`_start_waypoints`/`_cancel_navigation`, handled
        directly in `send()`); the real `nav_goal` handler
        (`command_handlers.py:30`) runs actual Nav2, so treating it as a
        stub here would be sim/robot behaviour the transport seam exists to
        prevent. Map/nav-mode bookkeeping below is just recording what was
        asked for, not simulating localization.
        """
        cmd_type = msg.get("type")

        if cmd_type == protocol.CMD_GET_NAV_MODE:
            return {
                "mode": self._nav_mode,
                "map": self._nav_map,
                "transitioning": False,
                "localized": self._nav_mode != "idle",
            }
        if cmd_type == protocol.CMD_ENTER_MAPPING_MODE:
            self._nav_mode, self._nav_map = "mapping", None
            return {"ok": True, "mode": self._nav_mode}
        if cmd_type == protocol.CMD_ENTER_NAVIGATION_MODE:
            name = msg.get("name")
            if name not in self._maps:
                return {
                    "ok": False,
                    "mode": self._nav_mode,
                    "error": f"no such map: {name!r}",
                }
            self._nav_mode, self._nav_map = "navigating", name
            return {"ok": True, "mode": self._nav_mode, "map": name}
        if cmd_type == protocol.CMD_STOP_NAV_MODE:
            self._nav_mode, self._nav_map = "idle", None
            return {"ok": True, "mode": self._nav_mode}

        if cmd_type == protocol.CMD_SAVE_MAP:
            name = msg.get("name") or "map"
            if name not in self._maps:
                self._maps.append(name)
            return {"ok": True, "name": name}
        if cmd_type == protocol.CMD_LOAD_MAP:
            name = msg.get("name")
            if name not in self._maps:
                return {"ok": False, "name": name}
            self._nav_map = name
            return {"ok": True, "name": name}
        if cmd_type == protocol.CMD_DELETE_MAP:
            name = msg.get("name")
            if name not in self._maps or name == self._nav_map:
                return {"ok": False, "name": name}
            self._maps.remove(name)
            return {"ok": True, "name": name}
        if cmd_type == protocol.CMD_LIST_MAPS:
            # Metadata dicts, not plain strings — mirrors the real server's
            # documented shape (PROTOCOL.md §5.2) so `list_maps()`'s
            # `name`-extraction is exercised against the sim too.
            return {"maps": [{"name": n, "size": 0, "modified": 0} for n in self._maps]}

        if cmd_type == protocol.CMD_LIST_LOCATIONS:
            return {"locations": []}  # 🔌 stub in v1, same as the real robot

        if cmd_type == protocol.CMD_HEALTH:
            return {"type": "health", "cpu": 0.0, "ram": 0.0, "temp": 0.0}
        if cmd_type == protocol.CMD_GET_SESSION_STATUS:
            return {
                "base": {
                    "running": True,
                    "owned": True,
                    "transitioning": False,
                    "error": None,
                },
                "nav": {"mode": self._nav_mode, "map": self._nav_map},
                "health": {"ok": True, "issues": []},
            }
        if cmd_type == protocol.CMD_SUBSCRIBE:
            return {"ok": True, "events": list(msg.get("events", []))}

        if cmd_type == protocol.CMD_SPEAK:
            # See `set_speech_provider` for the contract. No provider ->
            # the same silent ack this command always gave.
            if self._speech_provider is None:
                return {"ok": True}
            result = self._speech_provider(msg.get("text", ""), msg.get("voice"))
            return {"ok": True if result is None else bool(result)}

        # Everything else (set_initial_pose, start/stop_navigation,
        # start/stop_mapping, named locations, servo_single, head/display,
        # wifi/update, restart_base_session, ...) — a plain success ack is
        # the right shape for a v1 stub-or-inert command on a robot with
        # nothing physically behind it. nav_goal/navigate_through_waypoints/
        # cancel_nav are NOT here — see `send()`.
        return {"ok": True}
