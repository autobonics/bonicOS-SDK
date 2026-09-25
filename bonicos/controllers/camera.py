"""Camera (API.md §9).

Camera video is WebRTC under the hood, but the user never has to think about
lanes: on the native SDK the WebSocket transport transparently brings up a
WebRTC peer for video the first time a frame is requested; in the browser the
host is already streaming. Either way you just call ``get_frame``.

Frames are BGR ``numpy`` arrays (OpenCV's native layout). A multi-camera robot
(e.g. the M1's face and docking cameras) exposes each by name — ``list()``
returns them; pass one to ``get_frame`` or use ``get_frames`` for all at once.

If the transport genuinely can't carry video (the offline mock), or the video
extra isn't installed, camera calls raise
:class:`~bonicos.exceptions.CameraUnavailable`.
"""

from __future__ import annotations

from typing import Dict, List, Optional

from .. import protocol
from ..exceptions import CameraUnavailable
from ..transports.base import Frame
from ._base import ControllerBase


class CameraController(ControllerBase):
    def list(self) -> List[str]:
        """Names of the robot's cameras, from the connect handshake. Available
        on any transport (informational) — frames still need a video path."""
        return list(self._robot.cameras)

    def _names(self) -> List[str]:
        return list(self._robot.cameras) or ["main"]

    def start(self, cameras: Optional[List[str]] = None) -> None:
        """Bring the camera stream up now (otherwise it starts lazily on the
        first ``get_frame``). Blocks until the link is established. Raises
        :class:`CameraUnavailable` if this transport has no video path."""
        if not getattr(self._transport, "supports_camera", False):
            raise CameraUnavailable(
                f"the {type(self._transport).__name__} has no video path"
            )
        self._transport.start_camera(cameras or self._names())

    def get_frame(self, camera: Optional[str] = None) -> Optional[Frame]:
        """Latest frame (BGR ndarray) for ``camera`` (default: the first), or
        ``None`` if none has arrived yet. Starts the stream on first call."""
        self.start()
        return self._transport.read_frame(camera or self._names()[0])

    def get_frames(self) -> Dict[str, object]:
        """Latest frame for every camera, keyed by name (values may be
        ``None`` until each stream delivers its first frame)."""
        self.start()
        return {name: self._transport.read_frame(name) for name in self._names()}

    def stop(self) -> None:
        """Tear down the camera stream (idempotent). Commands/telemetry are
        unaffected — only the video path closes."""
        if getattr(self._transport, "supports_camera", False):
            self._transport.stop_camera()

    def pause(self, camera: Optional[str] = None) -> bool:
        """Stop the robot ENCODING video you aren't looking at, without
        dropping the stream.

        The robot attaches a track per camera when the video link comes up and
        has no way to know you've stopped reading frames — unwatched, that
        stream measured ~32% of a core on a real A2, on a board already short
        of headroom. Pausing drops it to a static frame a second.

        Cheaper and far faster than ``stop()``/``start()``: the track stays
        attached, so there is no renegotiation and ``resume()`` is instant.
        Use ``stop()`` when you're done with video altogether, ``pause()``
        when you'll want it back shortly.

        ``camera`` names one; omit it for all of them. Raises
        :class:`~bonicos.CommandError` if this connection carries no video at
        all (a local-WebSocket or BLE lane) or the robot has no camera by that
        name.
        """
        return self._set_enabled(False, camera)

    def resume(self, camera: Optional[str] = None) -> bool:
        """Undo :meth:`pause` — full frame rate again, no renegotiation."""
        return self._set_enabled(True, camera)

    def _set_enabled(self, enabled: bool, camera: Optional[str]) -> bool:
        payload: Dict[str, object] = {
            "type": protocol.CMD_SET_CAMERA_ENABLED,
            "enabled": enabled,
        }
        if camera is not None:
            payload["camera"] = camera
        self._command(payload)
        return True
