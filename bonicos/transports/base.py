"""The transport seam — the only interface ``robot.py`` depends on.

Every backend (``websocket``, ``webrtc``, ``sim``, ``mock``) implements this
``Protocol`` exactly as specified in ARCHITECTURE.md §2. ``robot.py`` never
imports a concrete transport module directly; it receives one already
constructed.
"""

from __future__ import annotations

from typing import Any, Optional, Protocol, runtime_checkable

#: Deferred feature (README.md "Scope of the first release") — every v1
#: transport's read_frame() returns None, so this is intentionally untyped
#: rather than pulling in numpy as a base dependency for an unused feature.
Frame = Any


@runtime_checkable
class Transport(Protocol):
    """Transport-agnostic interface ``BonicBot`` is built on."""

    def connect(self, timeout: float) -> dict:
        """Perform the handshake; return the ``auth_result`` payload.

        The payload (``robot_id``, ``series``, ``features``) lets
        ``BonicBot`` expose feature flags without knowing which transport is
        underneath.
        """
        ...

    def send(self, msg: dict) -> int:
        """Enqueue a command; return a monotonic client command id.

        The id correlates the eventual ``ack``/``error``. High-rate commands
        (``drive``) are still assigned an id for interface uniformity even
        though no reply is expected (see ``protocol.UNACKED_COMMANDS``).
        """
        ...

    def read_telemetry(self) -> dict:
        """Return the latest merged telemetry snapshot, non-blocking."""
        ...

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        """Block until the next telemetry frame arrives, or time out.

        Paces loops to the real sensor rate instead of spinning.
        """
        ...

    def wait_for_ack(self, cmd_id: int, timeout: float = 5.0) -> dict:
        """Block until the server acks/errors this command id."""
        ...

    #: Whether the SDK can obtain camera video over this transport. Camera is
    #: always WebRTC underneath: the browser (Pyodide) transport gets frames
    #: from the JS host, and the native WebSocket transport transparently
    #: brings up its OWN WebRTC peer for video (start_camera) — so both report
    #: ``True``. A transport with no video path at all (mock) reports ``False``
    #: so the camera API fails with a clear message instead of returning
    #: ``None`` forever.
    supports_camera: bool

    def start_camera(self, cameras: "list[str]") -> None:
        """Bring up the camera video path for ``cameras`` (idempotent).

        On the native transport this opens a WebRTC peer to the robot behind
        the scenes; in the browser runtime the host is already streaming and
        this is a no-op. Raises :class:`~bonicos.exceptions.CameraUnavailable`
        if video can't be established (missing extra deps, unreachable robot).
        """
        ...

    def stop_camera(self) -> None:
        """Tear down the camera video path if one is up (idempotent)."""
        ...

    def read_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        """Return the latest frame for ``camera`` (default: the first), or
        ``None`` if none has arrived yet. Call :meth:`start_camera` first.

        ``camera`` selects which stream on a multi-camera robot (names come
        from the connect handshake / :meth:`start_camera`).
        """
        ...

    def close(self) -> None:
        """Stop the robot (if applicable) and close the transport. Idempotent."""
        ...
