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

from ..exceptions import CameraUnavailable
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

    def get_frame(self, camera: Optional[str] = None):
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
