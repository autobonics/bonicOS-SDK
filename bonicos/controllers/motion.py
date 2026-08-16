"""Base movement (API.md §2) — high-rate wrappers over the ``drive`` command."""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Optional, Tuple

from .. import protocol
from ._base import ControllerBase

if TYPE_CHECKING:
    from ..robot import BonicBot


class MotionController(ControllerBase):
    #: Well under the on-robot 400ms cmd_vel deadman (PROTOCOL.md §7 / the
    #: robot_app ``core/deadman.py`` timeout) so continuous motion never
    #: gets zeroed out by the safety backstop.
    _DEADMAN_REFRESH_INTERVAL_S = 0.15

    def __init__(self, robot: "BonicBot") -> None:
        super().__init__(robot)
        self._lock = threading.Lock()
        self._current: Tuple[float, float] = (0.0, 0.0)
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_stop = threading.Event()

    def drive(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """Raw velocity (m/s, rad/s). Kept alive automatically while non-zero."""
        with self._lock:
            self._current = (linear_x, angular_z)
        self._send_raw(linear_x, angular_z)
        if linear_x == 0.0 and angular_z == 0.0:
            self._keepalive_stop.set()
        else:
            self._start_keepalive()

    def move_forward(
        self, speed: float = 0.3, duration: Optional[float] = None
    ) -> None:
        self.drive(linear_x=abs(speed))
        self._block_if_duration(duration)

    def move_backward(
        self, speed: float = 0.3, duration: Optional[float] = None
    ) -> None:
        self.drive(linear_x=-abs(speed))
        self._block_if_duration(duration)

    def turn_left(self, speed: float = 0.5, duration: Optional[float] = None) -> None:
        self.drive(angular_z=abs(speed))
        self._block_if_duration(duration)

    def turn_right(self, speed: float = 0.5, duration: Optional[float] = None) -> None:
        self.drive(angular_z=-abs(speed))
        self._block_if_duration(duration)

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def is_moving(self) -> bool:
        odom = self._latest(protocol.EVENT_ODOM)
        if not odom:
            return False
        return bool(odom.get("vx") or odom.get("vtheta"))

    # --- internal ------------------------------------------------------

    def _send_raw(self, linear_x: float, angular_z: float) -> None:
        self._send(
            {"type": protocol.CMD_DRIVE, "linear_x": linear_x, "angular_z": angular_z}
        )

    def _block_if_duration(self, duration: Optional[float]) -> None:
        if duration is not None:
            time.sleep(duration)
            self.stop()

    def _start_keepalive(self) -> None:
        if self._keepalive_thread is not None and self._keepalive_thread.is_alive():
            self._keepalive_stop.clear()
            return
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, daemon=True
        )
        try:
            self._keepalive_thread.start()
        except RuntimeError:
            # No real OS threads available — Pyodide's default single-
            # threaded WASM build (the only host that ever drives
            # SimTransport, whose "no threads" constraint applies
            # transitively here). The
            # keepalive exists solely to defeat a *real* robot's cmd_vel
            # deadman (PROTOCOL.md §7); SimTransport has none, so skipping it
            # changes nothing observable — the initial `_send_raw` above
            # already went out.
            self._keepalive_thread = None

    def _keepalive_loop(self) -> None:
        while not self._keepalive_stop.wait(self._DEADMAN_REFRESH_INTERVAL_S):
            with self._lock:
                linear_x, angular_z = self._current
            if linear_x == 0.0 and angular_z == 0.0:
                return
            self._send_raw(linear_x, angular_z)
