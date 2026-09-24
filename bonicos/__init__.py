"""``bonicos`` — one Python SDK for every BonicBot (see ../README.md).

Nothing heavy is imported here: :class:`BonicBot` imports its transport
lazily, inside the constructor, so ``import bonicos`` stays cheap. Computer
vision is a separate module, ``from bonicos import ai`` — see ``bonicos/ai``.
"""

from .enums import DisplayAnimation, HeadMode, ServoID
from .exceptions import (
    CommandError,
    ConnectionError,
    RobotDisconnected,
    RobotError,
)
from .protocol import PROTOCOL_VERSION
from .robot import BonicBot, use_transport

__version__ = "0.11.0"

__all__ = [
    "BonicBot",
    "use_transport",
    "HeadMode",
    "DisplayAnimation",
    "ServoID",
    "RobotError",
    "ConnectionError",
    "CommandError",
    "RobotDisconnected",
    "PROTOCOL_VERSION",
    "__version__",
]
