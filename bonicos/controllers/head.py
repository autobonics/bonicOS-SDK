"""Head expression & face display (API.md §6).

Expressions are live on every robot with a face display: the A-series LED
matrix and the S-series display. The ``display_*`` commands (text, colour,
animation, brightness, pixels) need the A-series matrix; the S display shows preset
expressions only and refuses them, as does a robot with no face display —
with an error, never a silent success (PROTOCOL.md §5.5).

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
from typing import Any, Dict, Iterable, List, Optional, Sequence, Union

from .. import protocol
from ..enums import DisplayAnimation, HeadMode
from ..exceptions import CommandError
from ._base import ControllerBase


def _plain(value: Any) -> Any:
    """A numpy array or scalar as plain Python, so it can go on the wire."""
    tolist = getattr(value, "tolist", None)
    return tolist() if callable(tolist) else value


def _number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _whole(value: Any) -> bool:
    return _number(value) and float(value).is_integer()


def _colour(value: Any) -> Optional[List[int]]:
    """``[r, g, b]`` as ints, or None if ``value`` is not three numbers."""
    value = _plain(value)
    if (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(_number(c) for c in value)
    ):
        return [int(c) for c in value]
    return None


class HeadController(ControllerBase):
    def set_expression(self, mode: Union[HeadMode, str]) -> bool:
        """Set the face — one of the :class:`HeadMode` expressions, each a
        real face on the A-series matrix and the S-series display.

        ``"none"`` blanks the A-series matrix; the S display always shows an
        expression and refuses it. An unknown name raises
        :class:`~bonicos.CommandError`, and ``CommandError.result["known"]``
        lists the names the robot knows.
        """
        value = mode.value if isinstance(mode, HeadMode) else mode
        self._command({"type": protocol.CMD_EMOTION, "emotion": value})
        return True

    def look(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None,
        speed: Optional[float] = None,
        *,
        duration: float = 1.0,
    ) -> bool:
        """Aim the neck. `pan`/`tilt` are DEGREES; at least one is required.

        Raises :class:`~bonicos.CommandError` when the robot has none of the
        axes asked for — for example `tilt` alone on an A2, which has no neck
        pitch. Asking for both on a robot with only one moves that one and
        warns about the other.

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
        # `unsupported` names URDF joints (neck_pitch_joint); report them as
        # this method's axis names.
        axis_of = {"neckYaw": "pan", "neckPitch": "tilt"}
        missing = [
            axis_of.get(protocol.REGISTRY_KEY_OF.get(j, j), j)
            for j in result.get("unsupported", [])
        ]
        if missing and len(missing) >= axes:
            raise CommandError(
                protocol.CMD_HEAD_LOOK,
                f"this robot's neck has no {' or '.join(missing)} joint",
                result,
            )
        if missing:
            warnings.warn(
                f"this robot's neck has no {' or '.join(missing)} joint — "
                "moved the rest",
                UserWarning,
                stacklevel=2,
            )
        return True

    def _display(self, msg: Dict[str, object]) -> bool:
        """Send one display command.

        A refusal — no LED matrix on this robot (the S display shows
        expressions only), the base stack down, an unknown animation name —
        raises :class:`~bonicos.CommandError` with
        the robot's reason. For an unknown animation,
        ``CommandError.result["known"]`` lists the names the robot knows.
        """
        self._command(msg)
        return True

    def set_display_text(self, text: str, mode: str = "scroll") -> bool:
        """Show text on the matrix.

        ``mode`` is ``"scroll"`` (the default) or ``"static"``. About three
        characters fit the panel at once, so longer text needs to scroll. Any
        other mode raises :class:`~bonicos.CommandError`.

        ASCII only — the panel's font has nothing else, and the robot
        replaces anything outside it rather than rendering a run of garbage
        glyphs.
        """
        name = str(mode).strip().lower()
        if name not in protocol.DISPLAY_TEXT_MODES:
            raise CommandError(
                protocol.CMD_DISPLAY_TEXT,
                f"unknown text mode: {mode!r}",
                {"known": sorted(protocol.DISPLAY_TEXT_MODES)},
            )
        return self._display(
            {"type": protocol.CMD_DISPLAY_TEXT, "text": text, "mode": name}
        )

    def set_display_pixel(self, x: int, y: int, r: int, g: int, b: int) -> bool:
        """Light one LED of the matrix drawing area: ``x`` 0-11, ``y`` 0-4
        from the top left, colour 0-255 per channel.

        The rest of the panel stays as it is, and any running animation
        stops, so pixels can be drawn one at a time; ``clear_display()``
        first to start from a blank panel. A position outside the drawing
        area raises :class:`~bonicos.CommandError`.
        """
        cmd = protocol.CMD_DISPLAY_PIXEL
        w, h = protocol.DISPLAY_WIDTH, protocol.DISPLAY_HEIGHT
        px, py = _plain(x), _plain(y)
        if not (_whole(px) and _whole(py) and 0 <= px < w and 0 <= py < h):
            raise CommandError(cmd, f"x must be 0-{w - 1} and y 0-{h - 1}")
        colour = _colour((r, g, b))
        if colour is None:
            raise CommandError(cmd, "r, g and b must be numbers, 0-255")
        return self._display(
            {
                "type": cmd,
                "x": int(px),
                "y": int(py),
                "r": colour[0],
                "g": colour[1],
                "b": colour[2],
            }
        )

    def set_display_frame(self, pixels: Sequence[Sequence[Any]]) -> bool:
        """Draw a whole picture on the matrix drawing area in one call.

        ``pixels`` is either 5 rows of 12 ``(r, g, b)`` colours, top row
        first, or all 60 colours in one flat list, row by row. ``(0, 0, 0)``
        is off. The panel is blanked first, so nothing from before shows
        around the picture. Any other shape raises
        :class:`~bonicos.CommandError`.
        """
        cmd = protocol.CMD_DISPLAY_FRAME
        w, h = protocol.DISPLAY_WIDTH, protocol.DISPLAY_HEIGHT
        data = _plain(pixels)
        iterable = isinstance(data, Iterable) and not isinstance(data, (str, bytes))
        items = [_plain(p) for p in data] if iterable else []
        if len(items) == h and all(
            isinstance(row, (list, tuple)) and len(row) == w for row in items
        ):
            items = [p for row in items for p in row]
        colours = [_colour(p) for p in items]
        if len(colours) != w * h or None in colours:
            raise CommandError(
                cmd,
                f"pixels must be {h} rows of {w} (r, g, b) colours, "
                f"or all {w * h} in one flat list",
            )
        return self._display({"type": cmd, "pixels": colours})

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
        """Blank the panel and stop whatever animation was running, so
        nothing repaints over the clear. Pixels drawn afterwards start from
        a blank panel."""
        return self._display({"type": protocol.CMD_DISPLAY_CLEAR})

    def set_display_brightness(self, value: float) -> bool:
        """Matrix brightness, 0-255 — NOT a 0..1 fraction. The firmware takes a
        raw FastLED brightness byte, so `0.5` is very nearly off."""
        return self._display({"type": protocol.CMD_DISPLAY_BRIGHTNESS, "value": value})
