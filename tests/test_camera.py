"""CameraController — the transparent WebRTC-for-video API and its gating.

No real WebRTC here (that's exercised end-to-end against the robot/sim); these
pin the controller's contract: it starts the video path lazily, routes by
camera name, and fails with a clear CameraUnavailable when a transport has no
video path.
"""

from __future__ import annotations

import pytest

from bonicos.controllers import CameraController
from bonicos.exceptions import CameraUnavailable
from bonicos.transports.mock import MockTransport


class FakeCameraTransport:
    """A transport that reports video support and echoes frames by name."""

    supports_camera = True

    def __init__(self):
        self.started_with = None
        self.stopped = False

    def start_camera(self, cameras):
        self.started_with = list(cameras)

    def read_frame(self, camera=None):
        return f"frame:{camera}"

    def stop_camera(self):
        self.stopped = True


class Robot:
    def __init__(self, transport, cameras):
        self._transport = transport
        self.cameras = list(cameras)


def test_list_returns_handshake_cameras():
    cam = CameraController(Robot(FakeCameraTransport(), ["face", "docking"]))
    assert cam.list() == ["face", "docking"]


def test_get_frame_starts_link_and_routes_by_name():
    tx = FakeCameraTransport()
    cam = CameraController(Robot(tx, ["face", "docking", "chest"]))

    assert cam.get_frame("docking") == "frame:docking"
    # started the video path lazily with the full camera list
    assert tx.started_with == ["face", "docking", "chest"]


def test_get_frame_defaults_to_first_camera():
    cam = CameraController(Robot(FakeCameraTransport(), ["face", "docking"]))
    assert cam.get_frame() == "frame:face"


def test_get_frames_returns_all_by_name():
    cam = CameraController(Robot(FakeCameraTransport(), ["face", "docking"]))
    assert cam.get_frames() == {"face": "frame:face", "docking": "frame:docking"}


def test_camera_unavailable_on_transport_without_video():
    # MockTransport has no video path → clear, gated failure, not a silent None.
    cam = CameraController(Robot(MockTransport(), ["face"]))
    with pytest.raises(CameraUnavailable):
        cam.get_frame()
    with pytest.raises(CameraUnavailable):
        cam.start()


def test_mock_transport_reports_no_camera():
    assert MockTransport().supports_camera is False


def test_websocket_transport_advertises_camera_support():
    # Native WS transport brings up WebRTC for video on demand, so it reports
    # support even though no frame exists until start_camera connects.
    from bonicos.transports.websocket import WebSocketTransport

    tx = WebSocketTransport("127.0.0.1", robot_id="X")
    assert tx.supports_camera is True
    assert tx.read_frame() is None            # nothing until start_camera
