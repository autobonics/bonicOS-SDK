"""Head expression & LED matrix (API.md §6) — all 🔌 stub in v1.

No ROS path exists yet for any of these (PROTOCOL.md §5.5); the server ships
log+no-op stub handlers so this API is complete and stable ahead of the
robot side. Methods still send real commands and block for their ack —
student code and the wire contract are both exercised even though the robot
doesn't move.
"""

from __future__ import annotations

from typing import Dict, Optional, Union

from .. import protocol
from ..enums import HeadMode
from ._base import ControllerBase


class HeadController(ControllerBase):
    def set_expression(self, mode: Union[HeadMode, str]) -> bool:
        value = mode.value if isinstance(mode, HeadMode) else mode
        result = self._command({"type": protocol.CMD_HEAD_MODE, "mode": value})
        return bool(result.get("ok", False))

    def look(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None,
        speed: Optional[float] = None,
    ) -> bool:
        payload: Dict[str, object] = {"type": protocol.CMD_HEAD_LOOK}
        if pan is not None:
            payload["pan"] = pan
        if tilt is not None:
            payload["tilt"] = tilt
        if speed is not None:
            payload["speed"] = speed
        result = self._command(payload)
        return bool(result.get("ok", False))

    def set_display_text(self, text: str) -> bool:
        result = self._command({"type": protocol.CMD_DISPLAY_TEXT, "text": text})
        return bool(result.get("ok", False))

    def set_display_color(self, r: int, g: int, b: int) -> bool:
        result = self._command(
            {"type": protocol.CMD_DISPLAY_COLOR, "r": r, "g": g, "b": b}
        )
        return bool(result.get("ok", False))

    def set_display_animation(self, mode: str) -> bool:
        result = self._command({"type": protocol.CMD_DISPLAY_ANIMATION, "mode": mode})
        return bool(result.get("ok", False))

    def play_display(self) -> bool:
        return self.set_display_animation("play")

    def pause_display(self) -> bool:
        return self.set_display_animation("pause")

    def clear_display(self) -> bool:
        result = self._command({"type": protocol.CMD_DISPLAY_CLEAR})
        return bool(result.get("ok", False))

    def set_display_brightness(self, value: float) -> bool:
        result = self._command(
            {"type": protocol.CMD_DISPLAY_BRIGHTNESS, "value": value}
        )
        return bool(result.get("ok", False))
