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
    """Camera frames were requested on a transport that can't carry video.

    Camera streaming is delivered as WebRTC media tracks, so it's only
    available when the SDK is connected over WebRTC (the in-browser runtime).
    Over the native WebSocket connection there is no video path — sensor
    telemetry (pose, IMU, joint states, …) still works, but camera does not.
    """

    def __init__(self, detail: str = "") -> None:
        msg = "camera streaming is only available over a WebRTC connection"
        super().__init__(f"{msg}: {detail}" if detail else msg)
