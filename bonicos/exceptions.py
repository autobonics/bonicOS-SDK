"""Exceptions raised across the ``bonicos`` public API (API.md §11)."""

from __future__ import annotations

from typing import Optional


class RobotError(Exception):
    """Base class for every exception ``bonicos`` raises."""


class ConnectionError(RobotError):
    """Connect or handshake failed."""


class CommandError(RobotError):
    """The robot refused a command, or tried it and failed.

    Raised for both refusal shapes on the wire (PROTOCOL.md §2): an ``error``
    response, and an ``ack`` whose ``ok`` is false.

    This covers *"this robot cannot do that"* as well as ordinary failures.
    Capability is not advertised or gated client-side (PROTOCOL.md §3.1), so a
    command a robot structurally cannot perform — docking on a robot with no
    docking addon — comes back as a refusal like any other, and the server's
    ``reason`` is what explains it.

    ``command`` is the command that was refused, ``reason`` the robot's
    explanation, and ``result`` the robot's whole reply — for example
    ``known`` names on an unknown animation, ``cameras`` on an unknown camera,
    ``failed`` per servo group.
    """

    def __init__(
        self, command: str, reason: str, result: Optional[dict] = None
    ) -> None:
        super().__init__(f"{command}: {reason}")
        self.command = command
        self.reason = reason
        self.result = dict(result or {})


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


class AIUnavailable(RobotError):
    """``bonicos.ai`` cannot run here at all.

    Raised with a sentence that says what to do, never at import time:
    ``from bonicos import ai`` always succeeds, so a program that merely
    imports it still runs in the simulator. The reasons are concrete — the
    program is running in the browser simulator (AI models run on a robot),
    the bundled models are not installed (``$BONICOS_MODELS_DIR`` unset, as on
    a laptop), or a runtime such as ``mediapipe`` is missing.
    """


class ModelNotFound(RobotError):
    """``ai.load(name)`` named a model that was not sent with this program.

    Trained models reach the robot alongside the program that uses them, so
    this almost always means a typo in the name, or a model that was deleted
    or renamed in the Train tab. ``available`` lists what *was* sent.
    """

    def __init__(self, name: str, available: list[str]) -> None:
        if available:
            sent = ", ".join(f'"{n}"' for n in sorted(available))
            detail = f"Models sent with this program: {sent}."
        else:
            detail = "No AI models were sent with this program."
        super().__init__(
            f'No AI model called "{name}". {detail} '
            "Check the name matches a model in the Train tab."
        )
        self.name = name
        self.available = list(available)


class InvalidModel(RobotError):
    """A trained model was found but cannot be run safely or correctly.

    The head file is damaged or does not match its description, it was
    trained against a different backbone build than this robot carries, or
    it asks for preprocessing this SDK does not implement. Running it anyway
    would produce confident nonsense rather than an error, so it is refused.
    """

    def __init__(self, name: str, reason: str) -> None:
        super().__init__(f'AI model "{name}" cannot be used: {reason}')
        self.name = name
        self.reason = reason
