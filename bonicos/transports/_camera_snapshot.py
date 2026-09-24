"""On-robot camera link — frames pulled over the control lane itself.

The sibling of ``_camera_link.py``, for the one place a WebRTC peer cannot
go: inside ``run_code``'s sandbox. That sandbox is ``bwrap --unshare-net`` —
a network namespace holding nothing but its own loopback, which is NOT the
host's. So a peer started there has no route to robot_app for signaling or
for media, and ICE has no candidate pair that could ever connect: the
sandbox offers ``127.0.0.1`` addresses reachable only inside itself, and
robot_app's own candidates aren't routable from within the namespace.
Moving signaling onto the unix socket would not change that — it would buy
an ICE timeout in place of a connection-refused.

What does cross is the unix socket robot_app already listens on (a socket is
a filesystem object, not a network one), which is exactly the lane the
WebSocket transport is already using here. So the frame comes back over the
protocol: ``get_camera_frame`` -> base64 JPEG -> one ``cv2.imdecode``.

That is less work than the media path, not more. robot_app's bridge already
holds the camera's JPEG undecoded, so this forwards bytes that exist;
WebRTC would decode that JPEG on the Pi, re-encode it to VP8, and decode it
again here. It needs no ``aiortc``, no ``av``, and no event loop — which is
also why it is the only camera path that works in the runner venv without
the ``[camera]`` extra.

Latest-frame semantics are unchanged: ``read_frame`` blocks only for the
round trip, and a poll faster than the camera publishes is answered
``unchanged`` and served from this object's cache.
"""

from __future__ import annotations

import base64
import threading
from typing import Any, Dict, Optional, Tuple

from .. import protocol
from ..exceptions import CameraUnavailable
from .base import Frame


class SnapshotCameraLink:
    """Pulls frames from robot_app one request at a time, over the transport's
    own control lane."""

    #: How long to wait for a frame ack. Generous next to the round trip it
    #: actually takes (a dict lookup and a base64 on the robot) because the
    #: alternative — a student's vision loop raising mid-run because the Pi
    #: was briefly busy — is worse than a slow frame.
    ACK_TIMEOUT = 10.0

    def __init__(self, transport: Any) -> None:
        self._transport = transport
        self._cameras: list[str] = []
        # Decoded ndarray + the seq it came from, per camera. The cache is
        # what makes `since_seq` worth quoting: an unchanged reply costs one
        # round trip and no decode at all.
        self._cache: Dict[str, Tuple[int, Any]] = {}
        # Serialises the request/response pair. Two threads sharing one
        # transport would otherwise interleave sends and each take the
        # other's ack — the ack map is keyed by command id, so the reply
        # would go to whichever thread asked for that id, not whichever is
        # waiting here.
        self._lock = threading.Lock()
        self._started = False

    # --- lifecycle ---------------------------------------------------------

    def start(self, cameras: list[str]) -> None:
        """Check the decode dependency once, up front (idempotent).

        Nothing is negotiated — there is no stream to bring up — but failing
        here matches the WebRTC link's contract: ``camera.start()`` raises if
        video cannot work, rather than every ``get_frame()`` returning None
        and leaving the caller to guess why.
        """
        if self._started:
            return
        try:
            import cv2  # type: ignore[import-not-found]  # noqa: F401
            import numpy  # noqa: F401
        except ImportError as exc:
            raise CameraUnavailable(
                f"decoding camera frames needs numpy and OpenCV ({exc})"
            ) from exc
        self._cameras = list(cameras) or ["main"]
        self._started = True

    def stop(self) -> None:
        self._cache = {}
        self._started = False

    # --- frames ------------------------------------------------------------

    def read_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        """Latest BGR ndarray for ``camera`` (default: first), or None if the
        robot has no frame for it yet."""
        name = camera or (self._cameras[0] if self._cameras else None)
        cached = self._cache.get(name) if name else None

        request: Dict[str, Any] = {"type": protocol.CMD_GET_CAMERA_FRAME}
        if name is not None:
            request["camera"] = name
        if cached is not None:
            request["since_seq"] = cached[0]

        with self._lock:
            cmd_id = self._transport.send(request)
            reply = self._transport.wait_for_ack(cmd_id, timeout=self.ACK_TIMEOUT)

        if reply.get("type") == protocol.TYPE_ERROR:
            error = str(reply.get("error", ""))
            if error.startswith("unknown_command"):
                # An older robot_app. Say which half is behind rather than
                # reporting this as a camera fault — the SDK here is new
                # enough and the board is not.
                raise CameraUnavailable(
                    "this robot_app is too old to serve camera frames over the "
                    "local socket — it needs the build that added "
                    f"`{protocol.CMD_GET_CAMERA_FRAME}`"
                )
            raise CameraUnavailable(f"camera frame request failed: {error}")
        if not reply.get("ok", False):
            raise CameraUnavailable(
                f"camera frame request failed: {reply.get('error', 'unknown error')}"
            )

        # Answer under the name the robot resolved, which is what `camera:
        # None` asked it to pick. Keying the cache on the request's `name`
        # instead would give the default camera two entries whose seqs drift.
        name = reply.get("camera") or name
        if reply.get("unchanged") and cached is not None:
            return cached[1]

        data = reply.get("data")
        if not data:
            # Configured but nothing published yet — a camera node still
            # coming up, or one that died. None is the documented answer.
            return None

        image = self._decode(data)
        if image is None:
            return cached[1] if cached is not None else None
        seq = reply.get("seq")
        if name is not None and isinstance(seq, int):
            self._cache[name] = (seq, image)
        return image

    @staticmethod
    def _decode(data: str) -> Optional[Any]:
        import cv2
        import numpy

        try:
            raw = base64.b64decode(data)
        except (ValueError, TypeError) as exc:
            raise CameraUnavailable(f"camera frame was not valid base64: {exc}") from exc
        # None on a truncated or corrupt JPEG. Treated as "no new frame"
        # by the caller rather than raised: one bad frame mid-stream should
        # not end a run that is otherwise working.
        return cv2.imdecode(numpy.frombuffer(raw, numpy.uint8), cv2.IMREAD_COLOR)
