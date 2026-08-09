"""Sanity-checks the SharedArrayBuffer bridge *mechanism* — encoding,
ring-buffer bookkeeping, seqlock framing — against a fake, single-threaded
``js``/``Atomics`` shim. This is **not** a real Pyodide/browser test (that
would need an actual worker + main-thread split); see webrtc.py's module
docstring for the "unverifiable in this environment" caveat from the plan.
"""

from __future__ import annotations

import json
import sys
import types

import pytest

from bonicos.exceptions import CameraUnavailable
from bonicos.transports import webrtc as webrtc_module
from bonicos.transports.webrtc import WebRTCTransport


class _FakeBuffer:
    def __init__(self, byte_length: int) -> None:
        self.byteLength = byte_length
        self.data = bytearray(byte_length)


class _Int32View:
    def __init__(self, buffer: _FakeBuffer) -> None:
        self._buffer = buffer

    def __getitem__(self, i: int) -> int:
        return int.from_bytes(self._buffer.data[i * 4 : i * 4 + 4], "little", signed=True)

    def __setitem__(self, i: int, value: int) -> None:
        self._buffer.data[i * 4 : i * 4 + 4] = int(value).to_bytes(4, "little", signed=True)


class _Uint8View:
    def __init__(self, buffer: _FakeBuffer) -> None:
        self._buffer = buffer

    def __getitem__(self, i: int) -> int:
        return self._buffer.data[i]

    def __setitem__(self, i: int, value: int) -> None:
        self._buffer.data[i] = value


class _FakeAtomics:
    @staticmethod
    def load(view: _Int32View, index: int) -> int:
        return view[index]

    @staticmethod
    def store(view: _Int32View, index: int, value: int) -> None:
        view[index] = value

    @staticmethod
    def add(view: _Int32View, index: int, delta: int) -> None:
        view[index] += delta

    @staticmethod
    def notify(view: _Int32View, index: int) -> None:
        pass

    @staticmethod
    def wait(view: _Int32View, index: int, expected: int, timeout_ms: int) -> str:
        # Single-threaded fake: data is always already there by the time
        # we'd block, so resolve immediately either way.
        return "ok" if view[index] == expected else "not-equal"


@pytest.fixture
def fake_js(monkeypatch):
    telemetry_buf = _FakeBuffer(4096)
    commands_buf = _FakeBuffer(64 * 1024)
    completions_buf = _FakeBuffer(4096)

    window = types.SimpleNamespace(
        __bonicosBridge__=types.SimpleNamespace(
            telemetry=telemetry_buf,
            commands=commands_buf,
            completions=completions_buf,
        )
    )

    fake_js = types.SimpleNamespace(
        window=window,
        Int32Array=types.SimpleNamespace(new=lambda buf: _Int32View(buf)),
        Uint8Array=types.SimpleNamespace(new=lambda buf: _Uint8View(buf)),
        Atomics=_FakeAtomics,
    )
    monkeypatch.setitem(sys.modules, "js", fake_js)
    return fake_js


#: Test camera layout: 2 cameras, 64-byte slots (16-byte header + room for a
#: small RGBA test image), matching FRAME_HEADER_BYTES.
_FRAME_SLOT_SIZE = 64
_TEST_CAMERAS = ["face", "docking"]


@pytest.fixture
def fake_js_with_cameras(fake_js):
    frames_buf = _FakeBuffer(_FRAME_SLOT_SIZE * len(_TEST_CAMERAS))
    fake_js.window.__bonicosBridge__.frames = frames_buf
    return fake_js


def _write_frame(fake_js, camera: str, width: int, height: int, rgba: bytes) -> None:
    """Write one seqlock frame into `camera`'s slot (odd-then-even seq bump,
    same protocol webrtc.py's _read_frame_slot expects)."""
    slot = _TEST_CAMERAS.index(camera)
    buf = fake_js.window.__bonicosBridge__.frames
    slot_offset = slot * _FRAME_SLOT_SIZE
    seq = int.from_bytes(
        buf.data[slot_offset : slot_offset + 4], "little", signed=True
    )
    buf.data[slot_offset : slot_offset + 4] = (seq + 1).to_bytes(
        4, "little", signed=True
    )
    buf.data[slot_offset + 4 : slot_offset + 8] = width.to_bytes(4, "little")
    buf.data[slot_offset + 8 : slot_offset + 12] = height.to_bytes(4, "little")
    buf.data[slot_offset + 12 : slot_offset + 16] = len(rgba).to_bytes(4, "little")
    start = slot_offset + webrtc_module.FRAME_HEADER_BYTES
    buf.data[start : start + len(rgba)] = rgba
    buf.data[slot_offset : slot_offset + 4] = (seq + 2).to_bytes(
        4, "little", signed=True
    )


def _write_telemetry_blob(fake_js, payload: dict) -> None:
    buf = fake_js.window.__bonicosBridge__.telemetry
    body = json.dumps(payload).encode("utf-8")
    buf.data[4:8] = len(body).to_bytes(4, "little")
    start = webrtc_module.TELEMETRY_HEADER_BYTES
    buf.data[start : start + len(body)] = body
    # seq stays even (0) = stable, matches the seqlock convention.


def test_connect_reads_auth_from_telemetry_blob(fake_js) -> None:
    _write_telemetry_blob(
        fake_js, {"_auth": {"robot_id": "SIM1", "series": "M", "features": {}}}
    )
    transport = WebRTCTransport()
    auth = transport.connect(timeout=1.0)
    assert auth["robot_id"] == "SIM1"


def test_send_writes_a_command_slot_and_advances_head(fake_js) -> None:
    _write_telemetry_blob(fake_js, {"_auth": {"robot_id": "SIM1"}})
    transport = WebRTCTransport()
    transport.connect(timeout=1.0)

    commands_buf = fake_js.window.__bonicosBridge__.commands
    head_before = int.from_bytes(commands_buf.data[0:4], "little")

    cmd_id = transport.send({"type": "drive", "linear_x": 0.5, "angular_z": 0.0})

    head_after = int.from_bytes(commands_buf.data[0:4], "little")
    assert head_after == head_before + 1

    slot_offset = webrtc_module.COMMAND_HEADER_BYTES + head_before * webrtc_module.COMMAND_SLOT_SIZE
    written_id = int.from_bytes(commands_buf.data[slot_offset : slot_offset + 4], "little")
    length = int.from_bytes(commands_buf.data[slot_offset + 4 : slot_offset + 8], "little")
    body = bytes(
        commands_buf.data[slot_offset + 8 : slot_offset + 8 + length]
    )
    decoded = json.loads(body.decode("utf-8"))
    assert written_id == cmd_id
    assert decoded == {"type": "drive", "linear_x": 0.5, "angular_z": 0.0}
    # drive is unacked — no "id" field on the wire payload itself.
    assert "id" not in decoded


def test_wait_for_ack_reads_a_completion_slot(fake_js) -> None:
    _write_telemetry_blob(fake_js, {"_auth": {"robot_id": "SIM1"}})
    transport = WebRTCTransport()
    transport.connect(timeout=1.0)

    cmd_id = transport.send({"type": "health"})

    completions_buf = fake_js.window.__bonicosBridge__.completions
    slot = cmd_id % transport._completions_slot_count
    slot_offset = slot * webrtc_module.COMPLETION_SLOT_SIZE
    body = json.dumps({"type": "ack", "id": cmd_id, "ok": True}).encode("utf-8")
    completions_buf.data[slot_offset + 4 : slot_offset + 8] = len(body).to_bytes(4, "little")
    completions_buf.data[slot_offset + 8 : slot_offset + 8 + len(body)] = body
    completions_buf.data[slot_offset : slot_offset + 4] = (1).to_bytes(4, "little")  # ready flag

    ack = transport.wait_for_ack(cmd_id, timeout=1.0)
    assert ack == {"type": "ack", "id": cmd_id, "ok": True}


# --- camera frames -----------------------------------------------------------

numpy = pytest.importorskip("numpy")


def _connect_with_cameras(fake_js_with_cameras):
    _write_telemetry_blob(
        fake_js_with_cameras, {"_auth": {"robot_id": "SIM1", "cameras": _TEST_CAMERAS}}
    )
    transport = WebRTCTransport()
    transport.connect(timeout=1.0)
    return transport


def test_supports_camera_is_true() -> None:
    assert WebRTCTransport.supports_camera is True


def test_read_frame_none_before_first_write(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    assert transport.read_frame("face") is None


def test_read_frame_decodes_rgba_to_bgr(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    # 2x2 RGBA: top-left red, top-right green, bottom-left blue, bottom-right white.
    rgba = bytes(
        [255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255]
    )
    _write_frame(fake_js_with_cameras, "face", width=2, height=2, rgba=rgba)

    frame = transport.read_frame("face")
    assert frame.shape == (2, 2, 3)
    assert list(frame[0, 0]) == [0, 0, 255]      # red pixel -> BGR
    assert list(frame[0, 1]) == [0, 255, 0]      # green pixel -> BGR (unchanged)
    assert list(frame[1, 0]) == [255, 0, 0]      # blue pixel -> BGR
    assert list(frame[1, 1]) == [255, 255, 255]  # white unaffected by channel swap


def test_read_frame_defaults_to_first_camera(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    rgba = bytes([1, 2, 3, 255] * 4)
    _write_frame(fake_js_with_cameras, "face", width=2, height=2, rgba=rgba)
    assert transport.read_frame() is not None


def test_read_frame_unknown_camera_returns_none(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    assert transport.read_frame("nonexistent") is None


def test_read_frame_none_without_frames_buffer(fake_js) -> None:
    # No `frames` key on the bridge at all — older/camera-less host build.
    _write_telemetry_blob(fake_js, {"_auth": {"robot_id": "SIM1", "cameras": ["face"]}})
    transport = WebRTCTransport()
    transport.connect(timeout=1.0)
    assert transport.read_frame("face") is None


def test_start_camera_returns_once_first_frame_lands(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    _write_frame(
        fake_js_with_cameras, "face", width=2, height=2, rgba=bytes([9, 9, 9, 255] * 4)
    )
    transport.start_camera(["face"], timeout=1.0)  # must not raise


def test_start_camera_times_out_without_a_frame(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    with pytest.raises(CameraUnavailable):
        transport.start_camera(["docking"], timeout=0.05)


def test_start_camera_raises_for_unknown_camera(fake_js_with_cameras) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    with pytest.raises(CameraUnavailable):
        transport.start_camera(["nonexistent"], timeout=0.05)


def test_start_camera_raises_without_frames_buffer(fake_js) -> None:
    _write_telemetry_blob(fake_js, {"_auth": {"robot_id": "SIM1", "cameras": ["face"]}})
    transport = WebRTCTransport()
    transport.connect(timeout=1.0)
    with pytest.raises(CameraUnavailable):
        transport.start_camera(["face"], timeout=0.05)


def test_decode_rgba_raises_camera_unavailable_without_numpy(
    fake_js_with_cameras, monkeypatch
) -> None:
    transport = _connect_with_cameras(fake_js_with_cameras)
    _write_frame(
        fake_js_with_cameras, "face", width=2, height=2, rgba=bytes([1, 2, 3, 255] * 4)
    )
    monkeypatch.setitem(sys.modules, "numpy", None)  # simulate "not installed"
    with pytest.raises(CameraUnavailable):
        transport.read_frame("face")
