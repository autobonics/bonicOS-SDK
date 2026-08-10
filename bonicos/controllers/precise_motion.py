"""Closed-loop precise motion (API.md §3) — client-side, v1 (dev/ARCHITECTURE.md §4).

Ported from ``bonicbot-bridge/precisemotion.py``'s ``_drive_distance_internal``
/ ``_rotate_angle_internal`` (read odom -> compute error -> send ``drive`` ->
repeat, distance via Euclidean delta from a captured start pose, rotation via
signed-yaw-delta accumulation to dodge wraparound), but paced by
``transport.wait_for_update()`` instead of a fixed ``time.sleep()`` poll, per
dev/ARCHITECTURE.md §4's loop pseudocode. The on-robot cmd_vel deadman is the
safety backstop for a stalled loop or dropped link (dev/ARCHITECTURE.md §4).
"""

from __future__ import annotations

import math
import queue
import threading
import time
from typing import TYPE_CHECKING, Iterable, Optional, Tuple

from .. import protocol
from ._base import ControllerBase

if TYPE_CHECKING:
    from ..robot import BonicBot


class PreciseMotionController(ControllerBase):
    DISTANCE_TOLERANCE_M = 0.02
    ANGLE_TOLERANCE_DEG = 1.0
    ODOM_WAIT_TIMEOUT_S = 5.0
    _UPDATE_POLL_TIMEOUT_S = 0.2

    def __init__(self, robot: "BonicBot") -> None:
        super().__init__(robot)
        self._queue: "queue.Queue[Tuple[str, float]]" = queue.Queue()
        self._queue_thread: Optional[threading.Thread] = None
        self._queue_done = threading.Event()
        self._queue_done.set()
        self._queue_cancel = threading.Event()
        self._queue_ok = True

    def drive_distance(
        self, meters: float, speed: float = 0.3, timeout: float = 30.0
    ) -> bool:
        odom = self._wait_for_odom(min(timeout, self.ODOM_WAIT_TIMEOUT_S))
        if odom is None:
            return False
        direction = 1.0 if meters >= 0 else -1.0
        target = abs(meters)
        cmd_speed = abs(speed) * direction
        start_x, start_y = odom["x"], odom["y"]
        deadline = time.monotonic() + timeout
        motion = self._robot.motion
        try:
            while True:
                if time.monotonic() >= deadline:
                    return False
                odom = self._latest(protocol.EVENT_ODOM)
                if odom is not None:
                    traveled = math.hypot(odom["x"] - start_x, odom["y"] - start_y)
                    if traveled >= target - self.DISTANCE_TOLERANCE_M:
                        return True
                motion.drive(linear_x=cmd_speed)
                self._transport.wait_for_update(self._UPDATE_POLL_TIMEOUT_S)
        finally:
            motion.stop()

    def rotate_angle(
        self, degrees: float, speed: float = 45.0, timeout: float = 30.0
    ) -> bool:
        odom = self._wait_for_odom(min(timeout, self.ODOM_WAIT_TIMEOUT_S))
        if odom is None:
            return False
        direction = 1.0 if degrees >= 0 else -1.0
        target = abs(degrees)
        cmd_speed = math.radians(abs(speed)) * direction
        last_theta = odom["theta"]
        accumulated = 0.0
        deadline = time.monotonic() + timeout
        motion = self._robot.motion
        try:
            while True:
                if time.monotonic() >= deadline:
                    return False
                odom = self._latest(protocol.EVENT_ODOM)
                if odom is not None:
                    theta = odom["theta"]
                    accumulated += self._normalize_angle(theta - last_theta)
                    last_theta = theta
                    if (
                        math.degrees(abs(accumulated))
                        >= target - self.ANGLE_TOLERANCE_DEG
                    ):
                        return True
                motion.drive(angular_z=cmd_speed)
                self._transport.wait_for_update(self._UPDATE_POLL_TIMEOUT_S)
        finally:
            motion.stop()

    def drive_and_rotate(
        self,
        meters: float,
        degrees: float,
        speed: float = 0.3,
        turn_speed: float = 45.0,
        timeout: float = 30.0,
    ) -> bool:
        half = timeout / 2
        return self.drive_distance(meters, speed, half) and self.rotate_angle(
            degrees, turn_speed, half
        )

    def draw_square(
        self, side_m: float, speed: float = 0.3, turn_speed: float = 45.0
    ) -> bool:
        for _ in range(4):
            if not self.drive_distance(side_m, speed):
                return False
            if not self.rotate_angle(90.0, turn_speed):
                return False
        return True

    # --- command queue ---------------------------------------------------

    def enqueue(self, cmd_list: Iterable[Tuple[str, float]]) -> None:
        for item in cmd_list:
            self._queue.put(item)

    def run_queue(self, block: bool = True) -> bool:
        self._queue_cancel.clear()
        self._queue_done.clear()
        self._queue_ok = True
        self._queue_thread = threading.Thread(target=self._drain_queue, daemon=True)
        self._queue_thread.start()
        if block:
            self._queue_done.wait()
            return self._queue_ok
        return True

    def clear_queue(self) -> None:
        self._queue_cancel.set()
        with self._queue.mutex:
            self._queue.queue.clear()
        self._robot.motion.stop()

    def _drain_queue(self) -> None:
        try:
            while not self._queue_cancel.is_set():
                try:
                    kind, value = self._queue.get_nowait()
                except queue.Empty:
                    break
                if kind == "drive":
                    ok = self.drive_distance(value)
                elif kind == "rotate":
                    ok = self.rotate_angle(value)
                else:
                    ok = False
                self._queue_ok = self._queue_ok and ok
        finally:
            self._queue_done.set()

    # --- internal ----------------------------------------------------------

    def _wait_for_odom(self, timeout: float = ODOM_WAIT_TIMEOUT_S) -> Optional[dict]:
        deadline = time.monotonic() + timeout
        odom = self._latest(protocol.EVENT_ODOM)
        while odom is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._transport.wait_for_update(min(remaining, 1.0))
            odom = self._latest(protocol.EVENT_ODOM)
        return odom

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
