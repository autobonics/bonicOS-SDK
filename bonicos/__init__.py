"""``bonicos`` — one Python SDK for every BonicBot (see ../README.md).

Nothing heavy is imported here: :class:`BonicBot` itself only imports
transport modules lazily, inside its constructor, once it knows which
environment it's running in (ARCHITECTURE.md §1).
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
from .robot import BonicBot

__version__ = "0.1.0"

__all__ = [
    "BonicBot",
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
