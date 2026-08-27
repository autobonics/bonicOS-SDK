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

    #: How often a timed move (``move_forward(speed, duration)`` and friends)
    #: lets the transport run while it waits out the duration. Matches the
    #: 60 Hz render pump an embedding host drives off these same calls, so a
    #: simulated move animates at frame rate instead of stepping.
    _DURATION_POLL_INTERVAL_S = 1 / 60

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
        """Hold the current velocity for ``duration``, then stop.

        Paced by ``wait_for_update()`` rather than one ``time.sleep(duration)``
        — the same reason ``precise_motion.py`` polls instead of sleeping. A
        bare sleep never gives the transport a chance to run, and
        ``SimTransport`` only advances its physics when it is called
        (``send``/``wait_for_update``/``read_telemetry`` all ``_tick()``): the
        whole move would then land in the single tick that ``stop()`` triggers,
        so a simulated robot teleports to the end pose instead of driving
        there. Against a real robot this loop is equally correct — it just
        blocks on telemetry instead of on the clock.
        """
        if duration is None:
            return
        deadline = time.monotonic() + duration
        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return
                slice_s = min(remaining, self._DURATION_POLL_INTERVAL_S)
                started = time.monotonic()
                self._transport.wait_for_update(slice_s)
                # Sleep out whatever is left of the slice. Both kinds of
                # transport can return early — a real one the moment a
                # telemetry frame lands, `SimTransport` immediately and
                # always ("one call, one tick", its module docstring's
                # constraint 1, which makes the timeout advisory) — and
                # without this the loop would spin as fast as the
                # interpreter allows for the whole duration.
                leftover = slice_s - (time.monotonic() - started)
                if leftover > 0:
                    time.sleep(leftover)
        finally:
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
