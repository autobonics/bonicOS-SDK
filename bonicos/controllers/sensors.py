"""Sensors & telemetry (API.md §8).

Telemetry is pushed continuously by the server and cached by the transport;
every getter here is non-blocking and returns the latest cached value.
Use ``wait_for_update()`` to pace a loop to the real sensor rate instead of
spinning (API.md §8 "Recommended loop pattern").
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Dict, Iterable, Optional, Tuple, Union

from .. import protocol
from ._base import ControllerBase

if TYPE_CHECKING:
    from ..robot import BonicBot

Position = Union[Tuple[float, float], Dict[str, float]]


class SensorsController(ControllerBase):
    def __init__(self, robot: "BonicBot") -> None:
        super().__init__(robot)
        self._traveled_baseline: Optional[Tuple[float, float]] = None

    def get_position(self) -> Dict[str, float]:
        event = self._latest(protocol.EVENT_POSE)
        if not event:
            return {"x": 0.0, "y": 0.0, "theta": 0.0}
        return {
            "x": event.get("x", 0.0),
            "y": event.get("y", 0.0),
            "theta": event.get("theta", 0.0),
        }

    def get_x(self) -> float:
        return self.get_position()["x"]

    def get_y(self) -> float:
        return self.get_position()["y"]

    def get_heading(self) -> float:
        return math.degrees(self.get_position()["theta"])

    def get_battery(self) -> float:
        event = self._latest(protocol.EVENT_BATTERY)
        return float(event.get("soc", 0.0)) if event else 0.0

    def get_imu(self) -> Dict[str, float]:
        event = self._latest(protocol.EVENT_IMU)
        if not event:
            return {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
        return {k: event.get(k, 0.0) for k in ("ax", "ay", "az", "gx", "gy", "gz")}

    def get_distance_traveled(self, start: Optional[Position] = None) -> float:
        """Odometry-derived distance from ``start`` (default: first odom seen)."""
        event = self._latest(protocol.EVENT_ODOM)
        if not event:
            return 0.0
        current = (event.get("x", 0.0), event.get("y", 0.0))
        if start is not None:
            if isinstance(start, dict):
                origin = (start.get("x", 0.0), start.get("y", 0.0))
            else:
                origin = (start[0], start[1])
        else:
            if self._traveled_baseline is None:
                self._traveled_baseline = current
            origin = self._traveled_baseline
        return math.hypot(current[0] - origin[0], current[1] - origin[1])

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        return self._transport.wait_for_update(timeout)

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        """Block until the first telemetry frame arrives after connect."""
        deadline = time.monotonic() + timeout
        if self._transport.read_telemetry():
            return True
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._transport.wait_for_update(min(remaining, 1.0))
            if self._transport.read_telemetry():
                return True

    def subscribe(self, events: Iterable[str]) -> bool:
        result = self._command(
            {"type": protocol.CMD_SUBSCRIBE, "events": list(events)}
        )
        return bool(result.get("ok", False))
