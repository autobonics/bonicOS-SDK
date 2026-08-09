"""Pyodide transport — the worker/Python side of the SharedArrayBuffer bridge.

Per ARCHITECTURE.md §3.2 / ``bonic-architecture.md`` §5: ``RTCPeerConnection``
is main-thread only, Pyodide runs in a Web Worker, and a student's
``while True:`` loop never yields — so the bridge to the peer connection must
be ``SharedArrayBuffer`` + ``Atomics``, never ``postMessage``. The host
(bonic.ai front end) owns the peer connection, creates the buffers, and hands
them to this module; this module only reads/writes them.

**Host contract (this build's assumption — pending alignment with the
bonic.ai front end).** ``bonic-architecture.md`` §5 gives an *illustrative*
byte layout (4 fixed ``float64`` sensor slots); the real ``bonicos``
telemetry is richer and dict-shaped (PROTOCOL.md §6: ``pose``, ``odom``,
``battery``, ``joint_states``, …), and per ARCHITECTURE.md §7 "keep [the
layout] in exactly one place" — the host, not this SDK. So rather than guess
a byte-exact layout unilaterally, this module implements the *mechanism*
(seqlock, command ring, completion slots, ``Atomics.wait``) generically over
serialized JSON, against a single documented entry point the host must
provide before Python starts:

    js.window.__bonicosBridge__ = {
        telemetry:   SharedArrayBuffer,  # seqlock: Int32[0]=seq (odd=writing,
                                          # even=stable); Int32[1]=byte length
                                          # of the UTF-8 JSON payload; JSON
                                          # bytes start at byte offset 8. The
                                          # payload is a dict merging every
                                          # cached event by name, e.g.
                                          # {"pose": {...}, "battery": {...}},
                                          # plus a reserved "_auth" key once
                                          # the handshake completes.
        commands:    SharedArrayBuffer,  # ring, 64KiB default: Int32[0]=head
                                          # (worker-owned), Int32[1]=tail
                                          # (main-thread-owned); fixed
                                          # COMMAND_SLOT_SIZE-byte slots from
                                          # byte offset 8, each slot =
                                          # int32 cmd_id + int32 length +
                                          # UTF-8 JSON command bytes.
        completions: SharedArrayBuffer,  # fixed slots keyed by cmd_id %
                                          # slot_count: Int32[0]=ready flag
                                          # (0=empty/1=ready, the
                                          # Atomics.wait target),
                                          # Int32[1]=byte length, UTF-8 JSON
                                          # ack/error payload from byte
                                          # offset 8.
        frames:      SharedArrayBuffer,  # OPTIONAL — omit entirely on a host
                                          # build with no camera support yet;
                                          # read_frame()/start_camera() then
                                          # degrade to "no video", not a crash.
                                          # One fixed-size seqlock slot per
                                          # camera, back-to-back, slot i
                                          # matching auth_result["cameras"][i]
                                          # (same order — no name directory
                                          # inside the buffer itself). Per
                                          # slot: Int32[0]=seq (odd=writing,
                                          # even=stable, bumped on every
                                          # write — same seqlock as
                                          # telemetry), Int32[1]=width,
                                          # Int32[2]=height, Int32[3]=byte
                                          # length of the pixel payload;
                                          # RGBA8 pixels (canvas ImageData's
                                          # native layout — cheapest for the
                                          # host to produce via
                                          # `ctx.getImageData()`, no
                                          # per-pixel channel work needed on
                                          # the JS side) start at byte offset
                                          # FRAME_HEADER_BYTES, row-major,
                                          # top-to-bottom. slot size =
                                          # ``frames.byteLength //
                                          # len(cameras)`` — the host picks
                                          # any resolution/slot size that
                                          # fits its cameras, the SDK derives
                                          # it rather than assuming one.
                                          # seq==0 (never written) reads as
                                          # "no frame yet", same as the
                                          # native transport's None.
        interrupt:   SharedArrayBuffer,  # handed to
                                          # pyodide.setInterruptBuffer() by
                                          # the host directly; this module
                                          # never touches it.
    }

``js`` / ``pyodide.ffi`` do not exist natively and are therefore imported
lazily, inside :meth:`WebRTCTransport.connect` only (ARCHITECTURE.md §1,
hard requirement).

**Camera frames come out as BGR ``numpy`` arrays**, matching the native
transport's contract (``camera.py``: "Frames are BGR numpy arrays, OpenCV's
native layout") — the host writes RGBA (its native, zero-copy format from
canvas), and :meth:`WebRTCTransport._decode_rgba` drops alpha and reverses
channel order with a single vectorized numpy slice, so the conversion cost
lands in Python (cheap, one call per frame) rather than the browser's video
pipeline (paid every frame, on the main thread, for every camera).
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

from .. import protocol
from ..exceptions import CameraUnavailable
from ..exceptions import ConnectionError as BonicConnectionError
from ..exceptions import RobotDisconnected

#: Default bridge global the host is expected to expose (README topology:
#: pro models only). Overridable via the constructor for e.g. ``sim.py``.
DEFAULT_BRIDGE_GLOBAL = "__bonicosBridge__"

COMMAND_HEADER_BYTES = 8  # Int32[0]=head, Int32[1]=tail
COMMAND_SLOT_SIZE = 512
COMMAND_SLOT_HEADER_BYTES = 8  # int32 cmd_id + int32 length
COMMAND_PAYLOAD_MAX = COMMAND_SLOT_SIZE - COMMAND_SLOT_HEADER_BYTES

COMPLETION_SLOT_SIZE = 512
COMPLETION_SLOT_HEADER_BYTES = 8  # int32 ready flag + int32 length
COMPLETION_PAYLOAD_MAX = COMPLETION_SLOT_SIZE - COMPLETION_SLOT_HEADER_BYTES

TELEMETRY_HEADER_BYTES = 8  # Int32[0]=seq, Int32[1]=length

FRAME_HEADER_BYTES = 16  # Int32[0]=seq, [1]=width, [2]=height, [3]=payload length
#: How long start_camera() waits for each requested camera's first frame
#: before raising CameraUnavailable — mirrors NativeCameraLink's default.
CAMERA_START_TIMEOUT_S = 15.0


class WebRTCTransport:
    """The Pyodide/SharedArrayBuffer side of the browser bridge.

    Constructed with no arguments in student code (``BonicBot()`` — see
    ARCHITECTURE.md §5); binds to the singleton the host preloaded rather
    than opening any connection of its own.
    """

    def __init__(self, *, bridge_global: str = DEFAULT_BRIDGE_GLOBAL) -> None:
        self._bridge_global = bridge_global
        self._js: Any = None
        self._telemetry_i32: Any = None
        self._telemetry_u8: Any = None
        self._commands_i32: Any = None
        self._commands_u8: Any = None
        self._commands_slot_count = 0
        self._completions_i32: Any = None
        self._completions_u8: Any = None
        self._completions_slot_count = 0
        self._frames_i32: Any = None
        self._frames_u8: Any = None
        self._frames_slot_size = 0
        self._camera_slot: Dict[str, int] = {}
        self._id_counter = 0
        self._auth_result: Dict[str, Any] = {}

    # --- Transport protocol ------------------------------------------------

    def connect(self, timeout: float = 10.0) -> dict:
        import js  # noqa: PLC0415 - lazy, Pyodide-only

        self._js = js

        bridge = getattr(js.window, self._bridge_global, None)
        if bridge is None:
            raise BonicConnectionError(
                f"js.window.{self._bridge_global} is not set — the host must"
                " create the peer connection and buffers before BonicBot() runs"
            )

        self._telemetry_i32 = js.Int32Array.new(bridge.telemetry)
        self._telemetry_u8 = js.Uint8Array.new(bridge.telemetry)

        self._commands_i32 = js.Int32Array.new(bridge.commands)
        self._commands_u8 = js.Uint8Array.new(bridge.commands)
        self._commands_slot_count = (
            bridge.commands.byteLength - COMMAND_HEADER_BYTES
        ) // COMMAND_SLOT_SIZE

        self._completions_i32 = js.Int32Array.new(bridge.completions)
        self._completions_u8 = js.Uint8Array.new(bridge.completions)
        self._completions_slot_count = (
            bridge.completions.byteLength // COMPLETION_SLOT_SIZE
        )

        self.send(
            {
                "type": protocol.TYPE_AUTH,
                "protocol_version": protocol.PROTOCOL_VERSION,
            }
        )

        deadline_ms = int(timeout * 1000)
        payload = self._read_telemetry_blob()
        waited_ms = 0
        while "_auth" not in payload:
            remaining_ms = deadline_ms - waited_ms
            if remaining_ms <= 0:
                raise BonicConnectionError("timed out waiting for auth_result")
            self._atomics_wait_telemetry(min(remaining_ms, 200))
            waited_ms += 200
            payload = self._read_telemetry_blob()

        self._auth_result = payload["_auth"]

        # Camera slot order is implicit: slot i <-> cameras[i] from this same
        # auth payload — no name directory needed inside the frames buffer.
        # `frames` is optional (older/camera-less host builds omit it), so
        # missing/empty just means no video, not a connect-time failure.
        cameras = list(self._auth_result.get("cameras") or [])
        frames_buf = getattr(bridge, "frames", None)
        if frames_buf is not None and cameras and frames_buf.byteLength > 0:
            self._camera_slot = {name: i for i, name in enumerate(cameras)}
            self._frames_i32 = js.Int32Array.new(frames_buf)
            self._frames_u8 = js.Uint8Array.new(frames_buf)
            self._frames_slot_size = frames_buf.byteLength // len(cameras)

        return dict(self._auth_result)

    def send(self, msg: dict) -> int:
        self._id_counter += 1
        cmd_id = self._id_counter

        payload = dict(msg)
        if payload.get("type") not in protocol.UNACKED_COMMANDS:
            payload["id"] = cmd_id

        body = json.dumps(payload).encode("utf-8")
        if len(body) > COMMAND_PAYLOAD_MAX:
            raise ValueError(
                f"command payload ({len(body)} bytes) exceeds the "
                f"{COMMAND_PAYLOAD_MAX}-byte ring slot"
            )

        head = self._js.Atomics.load(self._commands_i32, 0)
        tail = self._js.Atomics.load(self._commands_i32, 1)
        next_head = (head + 1) % self._commands_slot_count
        if next_head == tail:
            raise RobotDisconnected("command ring full — host isn't draining it")

        slot_offset = COMMAND_HEADER_BYTES + head * COMMAND_SLOT_SIZE
        self._write_u32(self._commands_u8, slot_offset, cmd_id)
        self._write_u32(self._commands_u8, slot_offset + 4, len(body))
        self._write_bytes(self._commands_u8, slot_offset + COMMAND_SLOT_HEADER_BYTES, body)

        self._js.Atomics.store(self._commands_i32, 0, next_head)
        self._js.Atomics.notify(self._commands_i32, 0)
        return cmd_id

    def read_telemetry(self) -> dict:
        payload = self._read_telemetry_blob()
        payload.pop("_auth", None)
        return payload

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        return self._atomics_wait_telemetry(int(timeout * 1000))

    def wait_for_ack(self, cmd_id: int, timeout: float = 5.0) -> dict:
        slot = cmd_id % self._completions_slot_count
        flag_index = (slot * COMPLETION_SLOT_SIZE) // 4
        deadline_ms = int(timeout * 1000)
        waited_ms = 0
        while True:
            ready = self._js.Atomics.load(self._completions_i32, flag_index)
            if ready == 1:
                slot_offset = slot * COMPLETION_SLOT_SIZE
                length = self._read_u32(self._completions_u8, slot_offset + 4)
                body = self._read_bytes(
                    self._completions_u8,
                    slot_offset + COMPLETION_SLOT_HEADER_BYTES,
                    length,
                )
                self._js.Atomics.store(self._completions_i32, flag_index, 0)
                result: Dict[str, Any] = json.loads(body.decode("utf-8"))
                if result.get("id") == cmd_id:
                    return result
                continue  # stale/unrelated completion, keep waiting
            remaining_ms = deadline_ms - waited_ms
            if remaining_ms <= 0:
                raise BonicConnectionError(
                    f"timed out waiting for ack of command {cmd_id}"
                )
            step_ms = min(remaining_ms, 200)
            self._js.Atomics.wait(self._completions_i32, flag_index, 0, step_ms)
            waited_ms += step_ms

    #: In the browser runtime the JS host owns the WebRTC peer and decodes
    #: video into the frames SAB, so video is available here too — as long as
    #: the host actually provided one (see connect(); an older/camera-less
    #: build just leaves start_camera()/read_frame() reporting no video).
    supports_camera = True

    def start_camera(
        self, cameras: list, timeout: float = CAMERA_START_TIMEOUT_S
    ) -> None:
        """Video is already streaming host-side (the peer connection predates
        BonicBot() entirely) — this only blocks until each requested
        camera's first frame lands, mirroring the native transport's
        "blocks until the link is established" contract instead of silently
        returning before any frame exists.
        """
        if self._frames_i32 is None:
            raise CameraUnavailable(
                "no `frames` buffer from the host — this browser session"
                " wasn't set up for camera streaming"
            )
        unknown = [c for c in cameras if c not in self._camera_slot]
        if unknown:
            raise CameraUnavailable(
                f"unknown camera(s): {', '.join(unknown)} — this robot has"
                f" {sorted(self._camera_slot)}"
            )
        deadline_ms = int(timeout * 1000)
        waited_ms = 0
        for camera in cameras:
            seq_index = (self._camera_slot[camera] * self._frames_slot_size) // 4
            while self._read_frame_slot(camera) is None:
                remaining_ms = deadline_ms - waited_ms
                if remaining_ms <= 0:
                    raise CameraUnavailable(
                        f"timed out waiting for '{camera}' video"
                    )
                step_ms = min(remaining_ms, 200)
                last_seq = self._js.Atomics.load(self._frames_i32, seq_index)
                self._js.Atomics.wait(self._frames_i32, seq_index, last_seq, step_ms)
                waited_ms += step_ms

    def stop_camera(self) -> None:
        return None  # host owns the peer connection; nothing here to tear down

    def read_frame(self, camera: Optional[str] = None):
        if camera is None:
            if not self._camera_slot:
                return None
            camera = next(iter(self._camera_slot))
        return self._read_frame_slot(camera)

    def close(self) -> None:
        self._js = None

    # --- frames (camera) ----------------------------------------------------

    def _read_frame_slot(self, camera: str) -> Optional[Any]:
        """Non-blocking seqlock read of one camera's slot -> BGR ndarray, or
        None if that camera is unknown or has never written a frame."""
        if self._frames_i32 is None:
            return None
        slot = self._camera_slot.get(camera)
        if slot is None:
            return None
        slot_offset = slot * self._frames_slot_size
        seq_index = slot_offset // 4
        while True:
            s1 = self._js.Atomics.load(self._frames_i32, seq_index)
            if s1 & 1:
                continue  # a write is in progress — retry
            if s1 == 0:
                return None  # never written yet
            width = self._read_u32(self._frames_u8, slot_offset + 4)
            height = self._read_u32(self._frames_u8, slot_offset + 8)
            length = self._read_u32(self._frames_u8, slot_offset + 12)
            body = self._read_bytes(
                self._frames_u8, slot_offset + FRAME_HEADER_BYTES, length
            )
            s2 = self._js.Atomics.load(self._frames_i32, seq_index)
            if s1 == s2:
                return self._decode_rgba(body, width, height)

    @staticmethod
    def _decode_rgba(body: bytes, width: int, height: int) -> Optional[Any]:
        """RGBA8 bytes (canvas ImageData layout) -> BGR ndarray, matching
        camera.py's "Frames are BGR numpy arrays" contract."""
        try:
            import numpy as np  # noqa: PLC0415 - lazy, only needed for camera use
        except ImportError as exc:
            raise CameraUnavailable(
                "camera frames need numpy — the host must preload the numpy"
                " Pyodide package"
            ) from exc
        if width <= 0 or height <= 0 or len(body) < width * height * 4:
            return None  # a torn/short write despite the seqlock check — skip it
        rgba = np.frombuffer(body, dtype=np.uint8).reshape(height, width, 4)
        return rgba[:, :, [2, 1, 0]]  # BGR, alpha dropped

    # --- seqlock / byte helpers ---------------------------------------------

    def _atomics_wait_telemetry(self, timeout_ms: int) -> bool:
        seq = self._js.Atomics.load(self._telemetry_i32, 0)
        outcome = self._js.Atomics.wait(self._telemetry_i32, 0, seq, timeout_ms)
        return str(outcome) != "timed-out"

    def _read_telemetry_blob(self) -> Dict[str, Any]:
        """Seqlock read that never returns a torn value (bonic-architecture.md §5)."""
        while True:
            s1 = self._js.Atomics.load(self._telemetry_i32, 0)
            if s1 & 1:  # odd = a write is in progress
                continue
            length = self._read_u32(self._telemetry_u8, 4)
            body = self._read_bytes(self._telemetry_u8, TELEMETRY_HEADER_BYTES, length)
            s2 = self._js.Atomics.load(self._telemetry_i32, 0)
            if s1 == s2:
                return json.loads(body.decode("utf-8")) if body else {}

    @staticmethod
    def _write_u32(u8_view: Any, byte_offset: int, value: int) -> None:
        for i in range(4):
            u8_view[byte_offset + i] = (value >> (8 * i)) & 0xFF

    @staticmethod
    def _read_u32(u8_view: Any, byte_offset: int) -> int:
        value = 0
        for i in range(4):
            value |= int(u8_view[byte_offset + i]) << (8 * i)
        return value

    @staticmethod
    def _write_bytes(u8_view: Any, byte_offset: int, data: bytes) -> None:
        for i, b in enumerate(data):
            u8_view[byte_offset + i] = b

    @staticmethod
    def _read_bytes(u8_view: Any, byte_offset: int, length: int) -> bytes:
        return bytes(u8_view[byte_offset + i] for i in range(length))
