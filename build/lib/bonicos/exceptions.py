"""Exceptions raised across the ``bonicos`` public API (API.md §11)."""

from __future__ import annotations


class RobotError(Exception):
    """Base class for every exception ``bonicos`` raises."""


class ConnectionError(RobotError):
    """Connect or handshake failed."""


class CommandError(RobotError):
    """The server returned an ``error`` response for a command."""

    def __init__(self, command: str, reason: str) -> None:
        super().__init__(f"{command}: {reason}")
        self.command = command
        self.reason = reason


class FeatureUnavailable(RobotError):
    """The requested feature is gated off for this robot's series."""

    def __init__(self, feature: str) -> None:
        super().__init__(f"feature not available on this robot: {feature!r}")
        self.feature = feature


class RobotDisconnected(RobotError):
    """The link dropped while a call was in flight or being awaited."""


class CameraUnavailable(RobotError):
    """Camera frames were requested but no video path could be established.

    Video is delivered as WebRTC media tracks, which the SDK brings up
    transparently over the existing connection the first time a frame is
    asked for (``transports/_camera_link.py``) — the caller never sets up a
    peer. So this is raised for a concrete reason, carried in ``detail``:
    the ``[camera]`` extra isn't installed (``pip install bonicos[camera]``),
    the named camera doesn't exist on this robot, the peer failed to
    negotiate, or the transport has no video path at all (``mock``).

    Driving, navigation, and sensor telemetry are unaffected either way.
    """

    def __init__(self, detail: str = "") -> None:
        msg = "camera video is unavailable"
        super().__init__(f"{msg}: {detail}" if detail else msg)
