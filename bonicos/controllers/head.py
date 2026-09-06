"""Head expression & LED matrix (API.md §6).

**Live on A series since robot_app gained the face-matrix path**; still stubs
on any robot whose series has no LED matrix, which answers with an error
rather than a silent success (PROTOCOL.md §5.5).

Angles are **degrees** at this API boundary, the same as `arm` (adopted
default #2), converted to radians here before the wire send — `head_look`
carries radians like every other joint command. This module used to forward
whatever number it was handed, which meant `look(pan=30)` asked for 30
*radians*; nothing acted on it while the server was a stub, and it is fixed
here rather than by changing the wire convention for one caller.
"""

from __future__ import annotations

import math
import warnings
from typing import Dict, Optional, Union

from .. import protocol
from ..enums import DisplayAnimation, HeadMode
from ._base import ControllerBase


class HeadController(ControllerBase):
    def set_expression(self, mode: Union[HeadMode, str]) -> bool:
        """Set the face.

        Two of the six expressions have no face in the robot's firmware and
        arrive as documented approximations — `surprised` is a heart,
        `confused` a colour effect. The server says so in the ack, and this
        raises a `UserWarning` rather than letting the substitution pass
        unnoticed: a lesson built around a "surprised" face should not
        discover on the day that it is a heart.
        """
        value = mode.value if isinstance(mode, HeadMode) else mode
        result = self._command({"type": protocol.CMD_HEAD_MODE, "mode": value})
        substituted = result.get("substituted")
        if substituted:
            warnings.warn(
                f"expression {value!r} has no face in firmware — showing "
                f"{substituted} instead",
                UserWarning,
                stacklevel=2,
            )
        return bool(result.get("ok", False))

    def look(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None,
        speed: Optional[float] = None,
        *,
        duration: float = 1.0,
    ) -> bool:
        """Aim the neck. `pan`/`tilt` are DEGREES; at least one is required.

        Returns False when the robot drove none of the axes asked for — `tilt`
        alone on an A2 is the case that matters, since A2 fits no neck pitch.
        Reporting True there is how a caller concludes the neck is broken
        rather than absent.

        `speed` is accepted for backwards compatibility and ignored by the
        server: ros2_control position groups take a time, not a rate. Use
        `duration` (seconds) instead.
        """
        payload: Dict[str, object] = {
            "type": protocol.CMD_HEAD_LOOK,
            "duration": duration,
        }
        axes = 0
        if pan is not None:
            payload["pan"] = math.radians(pan)
            axes += 1
        if tilt is not None:
            payload["tilt"] = math.radians(tilt)
            axes += 1
        if speed is not None:
            payload["speed"] = speed
        result = self._command(payload)
        if not result.get("ok", False):
            return False
        # Every axis we named came back undriven: the robot has none of them.
        return len(result.get("unsupported", [])) < axes

    def _display(self, msg: Dict[str, object]) -> bool:
        """Send one display command, surfacing the robot's reason for refusing.

        The display handlers answer a refusal — "no LED matrix on this
        series", the base stack being down, an animation name they do not
        know — as ``ok: False`` plus a plain-language ``error`` inside a
        NORMAL ack, not a protocol-level error, so ``_command`` returns it
        rather than raising and the bare ``bool`` these methods hand back
        would throw the sentence away. When the face stays dark, that
        sentence is the whole diagnosis, so it is re-raised as a warning
        instead of being swallowed.
        """
        result = self._command(msg)
        if result.get("ok", False):
            return True
        error = result.get("error")
        if error:
            warnings.warn(
                f"{msg.get('type', 'display command')} did nothing: {error}",
                UserWarning,
                stacklevel=3,
            )
        return False

    def set_display_text(self, text: str) -> bool:
        """Show text on the matrix.

        ASCII only — the panel's font has nothing else, and the robot
        replaces anything outside it rather than rendering a run of garbage
        glyphs.
        """
        return self._display({"type": protocol.CMD_DISPLAY_TEXT, "text": text})

    def set_display_color(self, r: int, g: int, b: int) -> bool:
        """Colour for text and colour-aware animations, 0-255 per channel."""
        return self._display(
            {"type": protocol.CMD_DISPLAY_COLOR, "r": r, "g": g, "b": b}
        )

    def set_display_animation(self, mode: Union[DisplayAnimation, str, int]) -> bool:
        """Play a named animation, or ``"play"``/``"pause"`` to control the
        current one.

        Names come from :class:`~bonicos.enums.DisplayAnimation`; a bare
        string works too, and a raw int is passed through as a firmware
        animation index for anything that enum has not named yet. An
        unknown name is refused by the robot with the list it does know.
        """
        value = mode.value if isinstance(mode, DisplayAnimation) else mode
        return self._display({"type": protocol.CMD_DISPLAY_ANIMATION, "mode": value})

    def play_display(self) -> bool:
        return self.set_display_animation("play")

    def pause_display(self) -> bool:
        return self.set_display_animation("pause")

    def clear_display(self) -> bool:
        """Blank the panel, and stop whatever animation was running — it
        drops the firmware into manual paint, so nothing immediately
        repaints over the clear."""
        return self._display({"type": protocol.CMD_DISPLAY_CLEAR})

    def set_display_brightness(self, value: float) -> bool:
        """Matrix brightness, 0-255 — NOT a 0..1 fraction. The firmware takes a
        raw FastLED brightness byte, so `0.5` is very nearly off."""
        return self._display({"type": protocol.CMD_DISPLAY_BRIGHTNESS, "value": value})
