"""SnapshotCameraLink — frames pulled over the control lane.

The video path the on-robot `run_code` sandbox gets, where `bwrap
--unshare-net` leaves no network namespace a WebRTC peer could use. These pin
the request/response contract against a fake transport: what is sent, what is
done with each reply shape, and that a poll faster than the camera publishes
costs no decode.
"""

from __future__ import annotations

import base64

import pytest

from bonicos import protocol
from bonicos.exceptions import CameraUnavailable
from bonicos.transports._camera_snapshot import SnapshotCameraLink

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")


def _jpeg(color: tuple[int, int, int]) -> bytes:
    """A real 8x8 JPEG, so the decode under test is the real cv2 one."""
    img = np.zeros((8, 8, 3), np.uint8)
    img[:, :] = color
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


class FakeTransport:
    """Records requests, replies from a scripted queue."""

    def __init__(self, replies: list[dict]) -> None:
        self.sent: list[dict] = []
        self._replies = list(replies)
        self._next_id = 0

    def send(self, msg: dict) -> int:
        self.sent.append(dict(msg))
        self._next_id += 1
        return self._next_id

    def wait_for_ack(self, cmd_id: int, timeout: float = 5.0) -> dict:
        return self._replies.pop(0)


def _link(replies: list[dict], cameras=("face", "docking")) -> SnapshotCameraLink:
    link = SnapshotCameraLink(FakeTransport(replies))
    link.start(list(cameras))
    return link


def _frame_reply(seq: int, color=(10, 20, 30), camera="face") -> dict:
    return {
        "type": protocol.TYPE_ACK, "ok": True, "camera": camera, "seq": seq,
        "encoding": "jpeg", "data": base64.b64encode(_jpeg(color)).decode(),
    }


# --- the request ---------------------------------------------------------


def test_asks_for_the_named_camera():
    link = _link([_frame_reply(1, camera="docking")])
    link.read_frame("docking")
    assert link._transport.sent[0]["type"] == protocol.CMD_GET_CAMERA_FRAME
    assert link._transport.sent[0]["camera"] == "docking"


def test_defaults_to_the_first_camera():
    link = _link([_frame_reply(1)])
    link.read_frame()
    assert link._transport.sent[0]["camera"] == "face"


def test_first_request_quotes_no_sequence_number():
    link = _link([_frame_reply(1)])
    link.read_frame()
    assert "since_seq" not in link._transport.sent[0]


def test_later_requests_quote_the_sequence_already_held():
    link = _link([_frame_reply(7), _frame_reply(8)])
    link.read_frame()
    link.read_frame()
    assert link._transport.sent[1]["since_seq"] == 7


# --- the replies ---------------------------------------------------------


def test_decodes_a_frame_to_a_bgr_array():
    link = _link([_frame_reply(1, color=(10, 20, 30))])
    frame = link.read_frame()
    assert frame.shape == (8, 8, 3)
    # JPEG is lossy, so pin the channel ordering rather than exact values.
    assert abs(int(frame[4, 4, 0]) - 10) < 12
    assert abs(int(frame[4, 4, 2]) - 30) < 12


def test_unchanged_serves_the_cache_without_decoding():
    link = _link([_frame_reply(3), {"type": protocol.TYPE_ACK, "ok": True,
                                    "camera": "face", "seq": 3,
                                    "unchanged": True}])
    first = link.read_frame()
    second = link.read_frame()
    # The same decoded object, not an equal one: an unchanged reply carries no
    # payload, so anything else would mean it decoded something.
    assert second is first


def test_no_frame_published_yet_is_none_not_an_error():
    link = _link([{"type": protocol.TYPE_ACK, "ok": True, "camera": "face",
                   "seq": 0, "data": None}])
    assert link.read_frame() is None


def test_a_corrupt_jpeg_falls_back_to_the_last_good_frame():
    corrupt = {"type": protocol.TYPE_ACK, "ok": True, "camera": "face",
               "seq": 2, "encoding": "jpeg",
               "data": base64.b64encode(b"not a jpeg").decode()}
    link = _link([_frame_reply(1), corrupt])
    good = link.read_frame()
    assert link.read_frame() is good


def test_a_corrupt_first_frame_is_none():
    link = _link([{"type": protocol.TYPE_ACK, "ok": True, "camera": "face",
                   "seq": 1, "encoding": "jpeg",
                   "data": base64.b64encode(b"not a jpeg").decode()}])
    assert link.read_frame() is None


def test_a_corrupt_frame_is_not_cached_as_seen():
    """Pins the sequence bookkeeping, not just the return value: caching seq 2
    against the *old* image would make the next poll say `unchanged` and hide
    the good frame that follows."""
    corrupt = {"type": protocol.TYPE_ACK, "ok": True, "camera": "face",
               "seq": 2, "encoding": "jpeg",
               "data": base64.b64encode(b"not a jpeg").decode()}
    link = _link([_frame_reply(1), corrupt, _frame_reply(3)])
    link.read_frame()
    link.read_frame()
    link.read_frame()
    assert link._transport.sent[2]["since_seq"] == 1


# --- failures ------------------------------------------------------------


def test_an_old_robot_app_says_so_by_name():
    link = _link([{"type": protocol.TYPE_ERROR,
                   "error": "unknown_command:get_camera_frame"}])
    with pytest.raises(CameraUnavailable, match="too old"):
        link.read_frame()


def test_an_unknown_camera_raises():
    link = _link([{"type": protocol.TYPE_ACK, "ok": False,
                   "error": "no camera 'nope'", "cameras": ["face"]}])
    with pytest.raises(CameraUnavailable, match="no camera"):
        link.read_frame()


def test_other_transport_errors_are_surfaced():
    link = _link([{"type": protocol.TYPE_ERROR, "error": "rate_limited"}])
    with pytest.raises(CameraUnavailable, match="rate_limited"):
        link.read_frame()


# --- lifecycle -----------------------------------------------------------


def test_start_is_idempotent_and_stop_clears_the_cache():
    link = _link([_frame_reply(5)])
    link.start(["face"])          # second call must not reset the camera list
    assert link._cameras == ["face", "docking"]
    link.read_frame()
    assert link._cache
    link.stop()
    assert not link._cache
