"""``bonicos`` — one Python SDK for every BonicBot (see ../README.md).

Nothing heavy is imported here: :class:`BonicBot` imports its transport
lazily, inside the constructor, so ``import bonicos`` stays cheap.
"""

from .enums import HeadMode, ServoID
from .exceptions import (
    CommandError,
    ConnectionError,
    FeatureUnavailable,
    RobotDisconnected,
    RobotError,
)
from .protocol import PROTOCOL_VERSION
from .robot import BonicBot, use_transport

__version__ = "0.1.0"

__all__ = [
    "BonicBot",
    "use_transport",
    "HeadMode",
    "ServoID",
    "RobotError",
    "ConnectionError",
    "CommandError",
    "FeatureUnavailable",
    "RobotDisconnected",
    "PROTOCOL_VERSION",
    "__version__",
]
