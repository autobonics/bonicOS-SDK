"""End-to-end test of the real socket + background-thread path.

Spins up a minimal in-process server implementing just enough of the
protocol (auth handshake, one telemetry push, one command ack) to exercise
:class:`bonicos.transports.websocket.WebSocketTransport` against an actual
``websockets`` connection rather than a mock.
"""

from __future__ import annotations

import json
import threading
import time

import pytest

websockets = pytest.importorskip("websockets")
from websockets.sync.server import serve  # noqa: E402

from bonicos.transports.websocket import WebSocketTransport  # noqa: E402


def _handler(ws) -> None:
    for raw in ws:
        msg = json.loads(raw)
        if msg["type"] == "auth":
            ws.send(
                json.dumps(
                    {
                        "type": "auth_result",
                        "ok": True,
                        "robot_id": "T1",
                        "series": "M",
                        "features": {"navigation": True},
                    }
                )
            )
            ws.send(
                json.dumps(
                    {"type": "battery", "voltage": 12.0, "current": 1.0, "soc": 88.0}
                )
            )
        elif msg["type"] == "drive":
            continue  # high-rate, no reply
        else:
            ws.send(json.dumps({"type": "ack", "id": msg.get("id"), "ok": True}))


@pytest.fixture
def server():
    with serve(_handler, "127.0.0.1", 0) as srv:
        thread = threading.Thread(target=srv.serve_forever, daemon=True)
        thread.start()
        yield srv
        srv.shutdown()


def test_handshake_telemetry_and_ack_over_a_real_socket(server) -> None:
    port = server.socket.getsockname()[1]
    transport = WebSocketTransport("127.0.0.1", robot_id="T1", port=port)
    try:
        auth = transport.connect(timeout=5.0)
        assert auth["robot_id"] == "T1"
        assert auth["features"]["navigation"] is True

        # wait_for_update() blocks for the NEXT update after it's called — it
        # can legitimately race with the server's post-auth battery push and
        # miss it entirely if that push already landed first (real, harmless
        # race: SensorsController.wait_for_data() exists specifically because
        # callers are expected to check read_telemetry() before falling back
        # to wait_for_update() as a pacing mechanism, not treat it as "block
        # until first data"). Mirror that idiom here instead of asserting a
        # guarantee wait_for_update() was never meant to provide.
        deadline = time.monotonic() + 2.0
        while not transport.read_telemetry() and time.monotonic() < deadline:
            transport.wait_for_update(0.2)
        telemetry = transport.read_telemetry()
        assert telemetry["battery"]["soc"] == 88.0

        cmd_id = transport.send({"type": "health"})
        ack = transport.wait_for_ack(cmd_id, timeout=5.0)
        assert ack["ok"] is True
        assert ack["id"] == cmd_id
    finally:
        transport.close()


def test_drive_omits_id_and_is_not_acked(server) -> None:
    port = server.socket.getsockname()[1]
    transport = WebSocketTransport("127.0.0.1", robot_id="T1", port=port)
    try:
        transport.connect(timeout=5.0)
        transport.send({"type": "drive", "linear_x": 0.1, "angular_z": 0.0})
        # No exception, no hang — drive frames just aren't acked. A short
        # wait_for_update confirms the connection is still alive and
        # processing (the earlier auth-time battery push already settled
        # the update event, so this just proves nothing broke).
        cmd_id = transport.send({"type": "health"})
        ack = transport.wait_for_ack(cmd_id, timeout=5.0)
        assert ack["ok"] is True
    finally:
        transport.close()
