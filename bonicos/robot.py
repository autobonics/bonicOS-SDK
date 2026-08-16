"""``BonicBot`` — the transport-agnostic user-facing facade (API.md).

The class body never mentions a transport (that's the whole point of the
:class:`~bonicos.transports.base.Transport` seam) — it builds one here, in
the constructor, and talks to the interface everywhere else. Tests swap in
``MockTransport`` through that same seam.

**There is exactly one transport to a real robot: the WebSocket lane**
(``ws://<host>:8080/ws``), on the developer's laptop and inside the
on-robot runner alike. Video is the sole exception, and it is invisible: a
WebRTC peer is negotiated behind the scenes the first time a frame is asked
for (``transports/_camera_link.py``), because media tracks are the only way
video leaves the robot. The caller never sets one up and never picks a
transport.

**Where the connection details come from — resolution order.** The goal is
that ``BonicBot()``, written exactly like that, is a working program in
every environment we ship, so the same file runs unchanged in a browser
simulator, in the on-robot runner, and on a laptop:

1. a transport registered by the host via :func:`use_transport` — the
   browser/Pyodide path, where there is no socket to open and the host
   supplies a simulator;
2. the ``host``/``robot_id`` arguments, if given — the laptop path;
3. ``$BONICOS_HOST`` / ``$BONICOS_ROBOT_ID`` — what the on-robot runner
   sets, so user code never hardcodes ``127.0.0.1``;
4. mDNS autodiscovery (optional ``discovery`` extra).

**``robot_id`` is optional, not a second required identifier.** ``host``
alone is enough to connect — it already names one specific machine.
``robot_id``, when given, does two things: on the local WS lane it becomes
a wrong-robot guard checked at handshake time (the server refuses the
connection if it doesn't match its own id — a safety net against a stale
IP after a DHCP change, not authentication); and when no ``host`` is given
at all, it narrows mDNS discovery to one robot on a LAN with several
advertising the service. Skip it entirely for the common case (one robot,
or you already trust ``host``).

This replaced an earlier ``sys.platform == "emscripten"`` check. Injection
is strictly better: it also covers the runner (step 3), and it keeps the
environment's knowledge in the environment instead of teaching the SDK to
recognise every place it might run.
"""

from __future__ import annotations

import os
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

from .controllers import (
    ArmController,
    CameraController,
    HeadController,
    MotionController,
    NavigationController,
    PreciseMotionController,
    SensorsController,
    SystemController,
)
from .enums import HeadMode
from .exceptions import ConnectionError as BonicConnectionError
from .transports.base import Frame, Transport

#: Transport registered by a host environment via :func:`use_transport`.
#: Module-level and persistent for the life of the interpreter: a browser
#: worker sets it once at startup, and every ``BonicBot()`` the user's code
#: constructs afterwards adopts it (one simulated robot per worker, which is
#: what a simulator should mean).
_injected_transport: Optional[Transport] = None


def use_transport(transport: Optional[Transport]) -> None:
    """Register a pre-built transport for :class:`BonicBot` to adopt.

    For **host environments**, not for user code. The browser runtime calls
    this with a simulator transport before running the user's program, so
    that a plain ``BonicBot()`` — with no host, no robot id, and no socket
    available — connects to the simulation instead of failing. Tests use it
    to inject :class:`~bonicos.transports.mock.MockTransport`.

    ``BonicBot()`` still calls ``connect()`` on whatever is registered, so a
    transport passed here must be unconnected and must return an
    ``auth_result``-shaped dict, exactly like the real one.

    Pass ``None`` to clear the registration.
    """
    global _injected_transport
    _injected_transport = transport


class BonicBot:
    """One SDK, every BonicBot. See ``API.md`` for the full method reference."""

    def __init__(
        self,
        host: Optional[str] = None,
        *,
        robot_id: Optional[str] = None,
        token: Optional[str] = None,
        timeout: float = 10.0,
    ) -> None:
        self._transport: Transport
        if _injected_transport is not None:
            # Host-supplied (browser simulator, tests). Nothing to resolve —
            # the environment already decided what this robot is.
            self._transport = _injected_transport
        else:
            from .transports.websocket import WebSocketTransport

            if host is None:
                host = os.environ.get("BONICOS_HOST") or None
            if robot_id is None:
                robot_id = os.environ.get("BONICOS_ROBOT_ID") or None

            if host is None:
                from .discovery import find_robot

                host = find_robot(robot_id, timeout)
                if host is None:
                    raise BonicConnectionError(
                        "no robot address given and mDNS discovery found none"
                        + (f" with robot_id={robot_id!r}" if robot_id else "")
                        + " — pass one explicitly, e.g. "
                        'BonicBot("192.168.1.50")'
                    )
            resolved_token = (
                token if token is not None else os.environ.get("BONICOS_TOKEN")
            )
            self._transport = WebSocketTransport(
                host, robot_id=robot_id, token=resolved_token
            )

        # The handshake carries **identity only** — no capability data
        # (PROTOCOL.md §3.1). Do not add `features`/`model`/`variant`/`joints`
        # back here: the SDK holds no model of what a robot can do, and a
        # command a robot cannot perform comes back as a CommandError with the
        # server's own explanation.
        auth_result = self._transport.connect(timeout)
        self.robot_id: str = auth_result.get("robot_id", robot_id or "")
        self.series: str = auth_result.get("series", "")
        #: Camera names this robot can stream (from the handshake). Frames come
        #: over WebRTC, which the transport sets up transparently (self.camera).
        #: An enumeration a client cannot guess — not a capability flag.
        self.cameras: List[str] = list(auth_result.get("cameras", []) or [])
        self._connected = True

        self.motion = MotionController(self)
        self.nav = NavigationController(self)
        self.arm = ArmController(self)
        self.head = HeadController(self)
        self.sensors = SensorsController(self)
        self.system = SystemController(self)
        self.camera = CameraController(self)
        self._precise = PreciseMotionController(self)

    @classmethod
    def simulated(
        cls,
        *,
        joints: Optional[Sequence[str]] = None,
    ) -> "BonicBot":
        """A fake robot in one line, for trying the SDK with no hardware.

        Shorthand for the ``use_transport``/``BonicBot()`` dance below —
        registers a fresh :class:`~bonicos.transports.sim.SimTransport` and
        connects to it immediately:

        .. code-block:: python

            import bonicos
            from bonicos.transports.sim import SimTransport

            bonicos.use_transport(SimTransport())
            robot = bonicos.BonicBot()

        Everything past this call is completely normal ``BonicBot`` code —
        driving, arms, telemetry — there is no simulator-specific API to
        learn (``robot.py`` cannot tell this transport from a real one).
        No Nav2/SLAM is simulated; navigation and mapping calls ack and do
        nothing, the same as their stub counterparts on real firmware.

        ``joints`` simulates a robot built with fewer than the full 18
        actuators — servo count is a per-robot build option, so it is worth
        being able to reproduce without hardware:

        .. code-block:: python

            robot = BonicBot.simulated(joints=["leftElbow", "neckYaw"])
            robot.get_servo_angles().keys()   # only those two

        There is no ``model`` parameter, and no simulated "lite" robot: the
        SDK models no capability at all (PROTOCOL.md §3.1), so there is
        nothing for a model name to select. To see what a real lite robot
        refuses, connect to one — the message comes from the robot.

        The registration is sticky for the interpreter's life, like
        :func:`use_transport` generally — a later bare ``BonicBot()`` adopts
        the same fake robot rather than looking for a real one, until you
        clear it with ``bonicos.use_transport(None)``.
        """
        from .transports.sim import SimTransport

        use_transport(SimTransport(joints=joints))
        return cls()

    # --- lifecycle (API.md §1) --------------------------------------------

    def is_connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        if not self._connected:
            return
        try:
            self.motion.stop()
        finally:
            self._transport.close()
            self._connected = False

    def __enter__(self) -> "BonicBot":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()

    # --- camera (API.md §9) -------------------------------------------------

    def list_cameras(self) -> List[str]:
        return self.camera.list()

    def get_camera_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        """Latest camera frame (BGR ndarray) or None. Video is brought up over
        WebRTC transparently on first use — see :class:`CameraController`."""
        return self.camera.get_frame(camera)

    # --- motion (API.md §2) -------------------------------------------------

    def drive(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        self.motion.drive(linear_x, angular_z)

    def move_forward(
        self, speed: float = 0.3, duration: Optional[float] = None
    ) -> None:
        self.motion.move_forward(speed, duration)

    def move_backward(
        self, speed: float = 0.3, duration: Optional[float] = None
    ) -> None:
        self.motion.move_backward(speed, duration)

    def turn_left(self, speed: float = 0.5, duration: Optional[float] = None) -> None:
        self.motion.turn_left(speed, duration)

    def turn_right(self, speed: float = 0.5, duration: Optional[float] = None) -> None:
        self.motion.turn_right(speed, duration)

    def stop(self) -> None:
        self.motion.stop()

    def is_moving(self) -> bool:
        return self.motion.is_moving()

    # --- precise motion (API.md §3) -----------------------------------------

    def drive_distance(
        self, meters: float, speed: float = 0.3, timeout: float = 30.0
    ) -> bool:
        return self._precise.drive_distance(meters, speed, timeout)

    def rotate_angle(
        self, degrees: float, speed: float = 45.0, timeout: float = 30.0
    ) -> bool:
        return self._precise.rotate_angle(degrees, speed, timeout)

    def drive_and_rotate(
        self,
        meters: float,
        degrees: float,
        speed: float = 0.3,
        turn_speed: float = 45.0,
        timeout: float = 30.0,
    ) -> bool:
        return self._precise.drive_and_rotate(
            meters, degrees, speed, turn_speed, timeout
        )

    def draw_square(
        self, side_m: float, speed: float = 0.3, turn_speed: float = 45.0
    ) -> bool:
        return self._precise.draw_square(side_m, speed, turn_speed)

    def enqueue(self, cmd_list: Iterable[Tuple[str, float]]) -> None:
        self._precise.enqueue(cmd_list)

    def run_queue(self, block: bool = True) -> bool:
        return self._precise.run_queue(block)

    def clear_queue(self) -> None:
        self._precise.clear_queue()

    # --- navigation, mapping & locations (API.md §4) ------------------------

    def go_to(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        wait: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        return self.nav.go_to(x, y, theta, wait, timeout)

    def navigate_waypoints(
        self, points: Sequence[Tuple[float, ...]], wait: bool = True
    ) -> bool:
        return self.nav.navigate_waypoints(points, wait)

    def cancel_goal(self) -> bool:
        return self.nav.cancel_goal()

    def wait_for_goal(self, timeout: float = 30.0) -> bool:
        return self.nav.wait_for_goal(timeout)

    def get_nav_status(self) -> str:
        return self.nav.get_nav_status()

    def get_distance_to_goal(self) -> float:
        return self.nav.get_distance_to_goal()

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0) -> bool:
        return self.nav.set_initial_pose(x, y, theta)

    def start_navigation(self) -> bool:
        return self.nav.start_navigation()

    def stop_navigation(self) -> bool:
        return self.nav.stop_navigation()

    def enter_mapping_mode(self, timeout: float = 30.0) -> bool:
        return self.nav.enter_mapping_mode(timeout)

    def enter_navigation_mode(self, name: str, timeout: float = 30.0) -> bool:
        return self.nav.enter_navigation_mode(name, timeout)

    def stop_nav_mode(self, timeout: float = 15.0) -> bool:
        return self.nav.stop_nav_mode(timeout)

    def get_nav_mode(self) -> Dict[str, Any]:
        return self.nav.get_nav_mode()

    def start_mapping(self) -> bool:
        return self.nav.start_mapping()

    def stop_mapping(self) -> bool:
        return self.nav.stop_mapping()

    def save_map(self, name: str = "map") -> bool:
        return self.nav.save_map(name)

    def load_map(self, name: str) -> bool:
        return self.nav.load_map(name)

    def delete_map(self, name: str) -> bool:
        return self.nav.delete_map(name)

    def list_maps(self) -> List[str]:
        return self.nav.list_maps()

    def get_map(self) -> Optional[Dict[str, Any]]:
        return self.nav.get_map()

    def save_location(self, name: str) -> bool:
        return self.nav.save_location(name)

    def goto_location(self, name: str, wait: bool = True) -> bool:
        return self.nav.goto_location(name, wait)

    def list_locations(self) -> List[str]:
        return self.nav.list_locations()

    def delete_location(self, name: str) -> bool:
        return self.nav.delete_location(name)

    def delete_all_locations(self) -> bool:
        return self.nav.delete_all_locations()

    # --- arms, grippers & neck (API.md §5) ----------------------------------

    def set_servos(
        self,
        angles: Dict[str, float],
        duration: float = 1.0,
        wait: bool = True,
        timeout: Optional[float] = None,
    ) -> bool:
        return self.arm.set_servos(angles, duration, wait=wait, timeout=timeout)

    def move_left_arm(
        self,
        shoulder: float,
        elbow: float,
        wait: bool = True,
        duration: float = 1.0,
        timeout: Optional[float] = None,
    ) -> bool:
        return self.arm.move_left_arm(shoulder, elbow, wait, duration, timeout)

    def move_right_arm(
        self,
        shoulder: float,
        elbow: float,
        wait: bool = True,
        duration: float = 1.0,
        timeout: Optional[float] = None,
    ) -> bool:
        return self.arm.move_right_arm(shoulder, elbow, wait, duration, timeout)

    def set_grippers(self, left: float, right: float) -> bool:
        return self.arm.set_grippers(left, right)

    def open_grippers(self) -> bool:
        return self.arm.open_grippers()

    def close_grippers(self) -> bool:
        return self.arm.close_grippers()

    def set_neck(self, yaw: float) -> bool:
        return self.arm.set_neck(yaw)

    def look_left(self) -> bool:
        return self.arm.look_left()

    def look_right(self) -> bool:
        return self.arm.look_right()

    def look_center(self) -> bool:
        return self.arm.look_center()

    def reset_servos(self) -> bool:
        return self.arm.reset_servos()

    def set_single_servo(self, joint: str, angle: float) -> bool:
        return self.arm.set_single_servo(joint, angle)

    def get_servo_angles(self) -> Dict[str, float]:
        return self.arm.get_servo_angles()

    # --- head expression & display (API.md §6) ------------------------------

    def set_expression(self, mode: Union[HeadMode, str]) -> bool:
        return self.head.set_expression(mode)

    def look(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        return self.head.look(pan, tilt, speed)

    def set_display_text(self, text: str) -> bool:
        return self.head.set_display_text(text)

    def set_display_color(self, r: int, g: int, b: int) -> bool:
        return self.head.set_display_color(r, g, b)

    def set_display_animation(self, mode: str) -> bool:
        return self.head.set_display_animation(mode)

    def play_display(self) -> bool:
        return self.head.play_display()

    def pause_display(self) -> bool:
        return self.head.pause_display()

    def clear_display(self) -> bool:
        return self.head.clear_display()

    def set_display_brightness(self, value: float) -> bool:
        return self.head.set_display_brightness(value)

    # --- speech (API.md §7) --------------------------------------------------

    def speak(self, text: str, voice: Optional[str] = None) -> bool:
        return self.system.speak(text, voice)

    # --- sensors & telemetry (API.md §8) ------------------------------------

    def get_position(self) -> Dict[str, float]:
        return self.sensors.get_position()

    def get_x(self) -> float:
        return self.sensors.get_x()

    def get_y(self) -> float:
        return self.sensors.get_y()

    def get_heading(self) -> float:
        return self.sensors.get_heading()

    def get_battery(self) -> float:
        return self.sensors.get_battery()

    def get_imu(self) -> Dict[str, float]:
        return self.sensors.get_imu()

    def get_distance_traveled(self, start: Optional[Any] = None) -> float:
        return self.sensors.get_distance_traveled(start)

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        return self.sensors.wait_for_update(timeout)

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        return self.sensors.wait_for_data(timeout)

    def subscribe(self, events: Iterable[str]) -> bool:
        return self.sensors.subscribe(events)

    # --- system (API.md §10) -------------------------------------------------

    def health(self) -> dict:
        return self.system.health()

    def restart_base_session(self, timeout: float = 120.0) -> bool:
        return self.system.restart_base_session(timeout)

    def get_session_status(self) -> Dict[str, Any]:
        return self.system.get_session_status()

    def reconfig_wifi(self, ssid: str, password: str) -> bool:
        return self.system.reconfig_wifi(ssid, password)

    def trigger_update(self) -> bool:
        return self.system.trigger_update()

    def ask_llm(self, prompt: str, model: Optional[str] = None) -> str:
        return self.system.ask_llm(prompt, model)
