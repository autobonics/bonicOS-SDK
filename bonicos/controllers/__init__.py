"""Feature-group controllers ``BonicBot`` delegates to (ARCHITECTURE.md §1).

Each controller is constructed by :class:`bonicos.robot.BonicBot` and takes
the ``BonicBot`` instance itself, not a bare transport — this lets e.g.
``PreciseMotionController`` call back into ``robot.motion`` without every
controller needing its own copy of motion logic (ARCHITECTURE.md §4).
"""

from .arm import ArmController
from .camera import CameraController
from .head import HeadController
from .motion import MotionController
from .navigation import NavigationController
from .precise_motion import PreciseMotionController
from .sensors import SensorsController
from .system import SystemController

__all__ = [
    "ArmController",
    "CameraController",
    "HeadController",
    "MotionController",
    "NavigationController",
    "PreciseMotionController",
    "SensorsController",
    "SystemController",
]
