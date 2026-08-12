"""Feature-group controllers ``BonicBot`` delegates to (keeps ``import bonicos`` cheap).

Each controller is constructed by :class:`bonicos.robot.BonicBot` and takes
the ``BonicBot`` instance itself, not a bare transport — this lets e.g.
``PreciseMotionController`` call back into ``robot.motion`` without every
controller needing its own copy of motion logic (dev/ARCHITECTURE.md §4).
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
