"""``SimTransport`` — a fake robot with no hardware behind it.

Built on :class:`~bonicos.transports.mock.MockTransport` for the id counter,
ack handling, and telemetry dict (dev/ARCHITECTURE.md §7) — but where
``MockTransport`` is a *test double* (telemetry the test sets by hand, acks
the test scripts), this is a *physics stand-in*: ``drive`` actually
integrates a pose, ``servo_command`` actually ramps joints toward their
targets. ``robot.py`` cannot tell this transport from a real one, which is
the whole point — a fake-robot mode built on the real SDK instead of a
reimplementation of it in another language is what makes sim/robot parity
structural rather than maintained.

**Two hard constraints, not style preferences (dev/ARCHITECTURE.md §3.2,
Implementation brief Task 5):**

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

Selection is the existing host-injection seam, nothing new
(dev/ARCHITECTURE.md §5)::

    import bonicos
    from bonicos.transports.sim import SimTransport

    bonicos.use_transport(SimTransport())
    robot = bonicos.BonicBot()        # everything below is unchanged

or the ``BonicBot.simulated()`` shortcut (API.md §1) for the common case of
one script, one fake robot, nothing else registered.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .. import protocol
from ..exceptions import CameraUnavailable
from .mock import MockTransport

#: A joint's animation state: (angle it started from, angle it's headed to,
#: the wall-clock time the move started, how long it should take) — all in
#: radians/seconds. Evaluated against elapsed time, never against a step
#: counter, for the same reason `drive` integrates against wall-clock time.
_Ramp = Tuple[float, float, float, float]


def _wrap_angle(theta: float) -> float:
    """Normalize to (-pi, pi] without a branchy modulo."""
    return math.atan2(math.sin(theta), math.cos(theta))


class SimTransport(MockTransport):
    """A simulated BonicBot: differential-drive pose + ramped servos.

    No Nav2, no SLAM — navigation/mapping commands ack success-shaped and do
    nothing, exactly matching the real robot's own stub convention for the
    commands that are still stubs there (Implementation brief Task 5). A
    little bookkeeping (which maps have been "saved", what nav mode is
    active) makes ``save_map``/``list_maps``/``enter_navigation_mode``/etc.
    behave sensibly for a demo without simulating path planning.
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
    ) -> None:
        """A fully-capable simulated robot.

        ``joints`` simulates a robot built with fewer than the full 18
        actuators — servo count is a per-robot build option, so this is a real
        case worth being able to reproduce without hardware.

        There is no ``model`` parameter: capability is not modelled anywhere in
        the SDK (PROTOCOL.md §3.1), so there is nothing for a model name to
        select. Navigation and mapping ack and do nothing here, matching the
        stub convention rather than a lite robot's hard failure.
        """
        super().__init__()
        fitted = (
            list(joints) if joints is not None else list(protocol.JOINT_NAME_MAP)
        )
        self._auth_result = {
            "robot_id": "SIM_001",
            "series": "SIM",
            "cameras": [],
        }
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

    # --- camera: no robot behind this transport, so no robot frames -----
    # (Implementation brief Task 5 — the browser's camera exercises use the
    # student's own webcam via the host, which this transport never sees.)

    supports_camera = False

    def start_camera(self, cameras: list) -> None:
        raise CameraUnavailable(
            "the simulator has no camera — in a browser, camera exercises "
            "use the student's own webcam via the host, not this transport"
        )

    def read_frame(self, camera: Optional[str] = None) -> None:
        return None

    # --- Transport protocol ----------------------------------------------

    def send(self, msg: dict) -> int:
        self._tick()  # settle everything up to "now" before this arrives
        cmd_type = msg.get("type")

        if cmd_type == protocol.CMD_DRIVE:
            self._linear_x = float(msg.get("linear_x", 0.0))
            self._angular_z = float(msg.get("angular_z", 0.0))
            return super().send(msg)  # unacked (protocol.UNACKED_COMMANDS)

        cmd_id = super().send(msg)
        if cmd_type == protocol.CMD_SERVO_COMMAND:
            result = self._start_servo_command(msg)
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
            self._integrate_drive(dt)
        self._advance_servos(now)
        self._publish_pose_and_odom()
        self._publish_joint_states()

    def _integrate_drive(self, dt: float) -> None:
        if self._linear_x == 0.0 and self._angular_z == 0.0:
            return
        self._x += self._linear_x * math.cos(self._theta) * dt
        self._y += self._linear_x * math.sin(self._theta) * dt
        self._theta = _wrap_angle(self._theta + self._angular_z * dt)

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

        Preemption matches real hardware (dev/ARCHITECTURE.md §4a, verified
        live): a joint already mid-ramp restarts from its *current
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
        # Precise-motion polls `odom`, not `pose` (dev/ARCHITECTURE.md §6) —
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

    # --- ack bookkeeping for everything that isn't drive/servo_command ----

    def _build_ack(self, msg: dict) -> dict:
        """Success-shaped acks for the rest of the command surface.

        Navigation/mapping commands acknowledge and otherwise do nothing —
        no Nav2, no SLAM, no `nav_status`/`plan` events — matching the real
        robot's own stub convention (Implementation brief Task 5). Map/
        nav-mode bookkeeping below is just recording what was asked for,
        not simulating planning or localization.
        """
        cmd_type = msg.get("type")

        if cmd_type == protocol.CMD_CANCEL_NAV:
            return {"canceled": True}

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

        # Everything else (nav_goal, navigate_through_waypoints,
        # set_initial_pose, start/stop_navigation, start/stop_mapping,
        # named locations, servo_single, head/display, speak, wifi/update,
        # restart_base_session, ...) — a plain success ack is the right
        # shape for a v1 stub-or-inert command on a robot with nothing
        # physically behind it.
        return {"ok": True}
