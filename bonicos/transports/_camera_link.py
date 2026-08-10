"""Native camera link — the WebRTC peer the WebSocket transport brings up
behind the scenes for video — media tracks are the only way it leaves the
robot.

The native SDK talks WebSocket for commands + telemetry. Video can't ride a
WebSocket, so the first time the user asks for a frame, the WebSocket
transport spins up THIS: a real ``aiortc`` peer to the robot's HTTP signaling
lane (``POST /webrtc/offer``), requests one recvonly video slot per camera,
decodes incoming frames to BGR ndarrays, and keeps the latest per camera. The
user never sees any of it — they just get frames.

Everything here is lazy: ``aiortc``/``av``/``numpy`` are only imported when a
link is actually started (``pip install bonicos[camera]``), so a student who
only drives the robot never pays for them. aiortc needs an asyncio loop and
the WebSocket transport has none (it's ``websockets.sync`` + a thread), so
this owns its own event loop on a dedicated daemon thread.
"""

from __future__ import annotations

import asyncio
import json
import threading
import urllib.request
from typing import Any, Dict, Optional

from ..exceptions import CameraUnavailable
from .base import Frame


class NativeCameraLink:
    """One WebRTC peer to the robot, receiving N camera tracks."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._cameras: list[str] = []

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._pc: Any = None

        # Latest BGR frame per transceiver mid, plus the {name: mid} map the
        # robot sends over the control channel. Dict rebinds are atomic under
        # the GIL, so readers need no lock (same pattern as the WS transport).
        self._frames_by_mid: Dict[str, Any] = {}
        self._name_to_mid: Dict[str, str] = {}

        self._connected = threading.Event()
        self._error: Optional[str] = None

    # --- lifecycle (called from the caller's thread) -----------------------

    def start(self, cameras: list[str], timeout: float = 15.0) -> None:
        if self._thread is not None:
            return  # idempotent
        try:
            import aiortc  # type: ignore[import-not-found]  # noqa: F401
            import av  # type: ignore[import-not-found]  # noqa: F401
            import numpy  # noqa: F401
        except ImportError as exc:
            raise CameraUnavailable(
                f"camera support needs extra dependencies — "
                f"`pip install bonicos[camera]` ({exc})"
            ) from exc

        self._cameras = list(cameras) or ["main"]
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        if not self._connected.wait(timeout):
            self.stop()
            raise CameraUnavailable(
                self._error or "timed out establishing the camera WebRTC link"
            )
        if self._error:
            self.stop()
            raise CameraUnavailable(self._error)

    def read_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        """Latest BGR ndarray for ``camera`` (default: first), or None."""
        name = camera or (self._cameras[0] if self._cameras else None)
        mid = self._name_to_mid.get(name) if name else None
        if mid is None:
            # Mapping not in yet (control message races the first frames). If
            # there's exactly one stream, it's unambiguous — hand it back.
            if len(self._frames_by_mid) == 1:
                return next(iter(self._frames_by_mid.values()))
            return None
        return self._frames_by_mid.get(mid)

    def stop(self) -> None:
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._shutdown)
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=3.0)
        self._thread = None
        self._loop = None

    # --- event-loop thread -------------------------------------------------

    def _run(self) -> None:
        assert self._loop is not None
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._negotiate())
        except Exception as exc:  # noqa: BLE001 - surfaced via CameraUnavailable
            self._error = str(exc)
            self._connected.set()
            return
        self._loop.run_forever()

    async def _negotiate(self) -> None:
        from aiortc import RTCPeerConnection, RTCSessionDescription

        pc = RTCPeerConnection()
        self._pc = pc

        control = pc.createDataChannel("control")

        @control.on("open")
        def _on_open() -> None:
            # Authenticate like any peer so the robot's unauthenticated-peer
            # reaper doesn't drop this video-only link (token is ignored by
            # robot_app — access is gated platform-side).
            control.send(json.dumps({"type": "auth"}))

        @control.on("message")
        def _on_control(raw: Any) -> None:  # camera_tracks maps mid -> name
            try:
                data = json.loads(raw)
            except (ValueError, TypeError):
                return
            if data.get("type") == "camera_tracks":
                self._name_to_mid = dict(data.get("tracks", {}))

        for _ in self._cameras:
            pc.addTransceiver("video", direction="recvonly")

        @pc.on("track")
        def _on_track(track: Any) -> None:
            asyncio.ensure_future(self._consume(track))

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)  # aiortc gathers ICE fully here
        assert self._loop is not None  # set by start() before this coroutine runs
        answer_sdp = await self._loop.run_in_executor(
            None, self._post_offer, pc.localDescription.sdp
        )
        await pc.setRemoteDescription(
            RTCSessionDescription(sdp=answer_sdp, type="answer")
        )
        self._connected.set()

    async def _consume(self, track: Any) -> None:
        mid = self._mid_of(track)
        while True:
            try:
                frame = await track.recv()
            except Exception:  # noqa: BLE001 - track ended / peer closed
                return
            if mid is None:  # mid can lag the first frame
                mid = self._mid_of(track)
            self._frames_by_mid[mid or track.id] = frame.to_ndarray(format="bgr24")

    def _mid_of(self, track: Any) -> Optional[str]:
        for t in self._pc.getTransceivers():
            if t.receiver is not None and t.receiver.track is track:
                mid = t.mid
                return str(mid) if mid is not None else None
        return None

    def _post_offer(self, sdp: str) -> str:
        body = json.dumps({"sdp": sdp, "sessionId": "bonicos-sdk-cam"}).encode()
        req = urllib.request.Request(
            f"http://{self._host}:{self._port}/webrtc/offer",
            data=body,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=15) as resp:
            return str(json.loads(resp.read().decode())["sdp"])

    def _shutdown(self) -> None:
        if self._pc is not None:
            asyncio.ensure_future(self._pc.close())
        assert self._loop is not None  # only scheduled while a loop is running
        self._loop.stop()
