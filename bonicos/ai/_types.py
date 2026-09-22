"""What ``bonicos.ai`` returns, plus the frame check every entry point shares.

Plain frozen dataclasses rather than dicts: ``obj.label`` autocompletes and a
typo is an ``AttributeError`` naming the field, which a student can act on. A
``KeyError`` from a dict is far less helpful. Coordinates are integer pixels in
the frame that was passed in, origin top-left, so they can be handed straight
to ``cv2.rectangle``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, NamedTuple, Tuple

from ..exceptions import AIUnavailable

Point = Tuple[int, int]


class Prediction(NamedTuple):
    """One class from a trained model, e.g. ``Prediction("cup", 0.94)``.

    A tuple, so ``label, confidence = model.predict(frame)[0]`` works."""

    label: str
    confidence: float


# Keyword-only so a subclass can add fields without inventing defaults for them.
@dataclass(frozen=True, kw_only=True)
class _Box:
    x: int
    y: int
    width: int
    height: int

    @property
    def center(self) -> Point:
        return (self.x + self.width // 2, self.y + self.height // 2)


@dataclass(frozen=True, kw_only=True)
class Detection(_Box):
    """An object found by :func:`bonicos.ai.detect_objects`."""

    label: str
    confidence: float


@dataclass(frozen=True, kw_only=True)
class Face(_Box):
    """A face found by :func:`bonicos.ai.detect_faces`. ``landmarks`` holds
    ``right_eye``, ``left_eye``, ``nose``, ``mouth_right`` and ``mouth_left`` —
    the person's own right and left."""

    confidence: float
    landmarks: Dict[str, Point]


@dataclass(frozen=True)
class Marker:
    """An ArUco marker found by :func:`bonicos.ai.detect_markers`. ``corners``
    run clockwise from the marker's own top-left corner."""

    id: int
    corners: Tuple[Point, Point, Point, Point]

    @property
    def center(self) -> Point:
        xs = [c[0] for c in self.corners]
        ys = [c[1] for c in self.corners]
        return (sum(xs) // 4, sum(ys) // 4)


@dataclass(frozen=True, kw_only=True)
class Gesture(_Box):
    """A hand found by :func:`bonicos.ai.detect_gestures`.

    ``name`` is one of ``Thumb_Up``, ``Thumb_Down``, ``Open_Palm``,
    ``Closed_Fist``, ``Pointing_Up``, ``Victory``, ``ILoveYou`` — or ``None``
    (the string) when a hand is visible but making no known gesture. ``hand``
    is the person's own ``"Left"`` or ``"Right"``. ``landmarks`` are the 21
    MediaPipe hand points, wrist first. The box is the landmarks' extent."""

    name: str
    confidence: float
    hand: str
    landmarks: Tuple[Point, ...]


def cv2_module() -> Any:
    try:
        import cv2
    except ImportError:
        raise AIUnavailable(
            "bonicos.ai needs OpenCV (opencv-python-headless), which is "
            "installed on the robot."
        ) from None
    return cv2


def as_bgr(frame: Any) -> Any:
    """The frame as a contiguous ``uint8`` BGR array, or a ``ValueError`` that
    says what went wrong — most often a ``None`` frame read before the camera
    delivered its first one."""
    if frame is None:
        raise ValueError(
            "frame is None — the camera has not delivered a frame yet. "
            "Check `if frame is None: continue` before using it."
        )
    import numpy as np

    cv2 = cv2_module()
    a = np.asarray(frame)
    if a.dtype != np.uint8:
        raise ValueError(
            f"expected a camera frame of 8-bit pixels (uint8), got {a.dtype}"
        )
    if a.ndim == 2:
        a = cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    elif a.ndim == 3 and a.shape[2] == 4:
        a = cv2.cvtColor(a, cv2.COLOR_BGRA2BGR)
    elif not (a.ndim == 3 and a.shape[2] == 3):
        raise ValueError(
            "expected a camera frame (height x width x 3, BGR), got an array of "
            f"shape {a.shape}"
        )
    return np.ascontiguousarray(a)
