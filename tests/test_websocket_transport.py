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
import urllib.parse

import pytest

websockets = pytest.importorskip("websockets")
from websockets.sync.server import serve  # noqa: E402

from bonicos import protocol  # noqa: E402
from bonicos.exceptions import ConnectionError as BonicConnectionError  # noqa: E402
from bonicos.exceptions import RobotDisconnected  # noqa: E402
from bonicos.transports.websocket import WebSocketTransport  # noqa: E402

#: Mirrors robot_app's core/local_ws.py v0 policy: robotId is optional, but
#: if the client sends one it must match this server's own id, or the
#: connection is refused with CLOSE_CODE_WRONG_ROBOT.
_SERVER_ROBOT_ID = "T1"


def _handler(ws) -> None:
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(ws.request.path).query)
    robot_id = query.get("robotId", [None])[0]
    if robot_id is not None and robot_id != _SERVER_ROBOT_ID:
        ws.close(code=protocol.CLOSE_CODE_WRONG_ROBOT)
        return

    for raw in ws:
        msg = json.loads(raw)
        if msg["type"] == "auth":
            ws.send(
                json.dumps(
                    {
                        "type": "auth_result",
                        "ok": True,
                        "robot_id": _SERVER_ROBOT_ID,
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


def test_build_url_omits_robot_id_query_param_by_default() -> None:
    transport = WebSocketTransport("127.0.0.1", port=1234)
    assert transport._build_url() == "ws://127.0.0.1:1234/ws"


def test_build_url_includes_robot_id_query_param_when_given() -> None:
    transport = WebSocketTransport("127.0.0.1", robot_id="M1_001", port=1234)
    assert transport._build_url() == "ws://127.0.0.1:1234/ws?robotId=M1_001"


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
    transport = WebSocketTransport("127.0.0.1", port=port)
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


def test_matching_robot_id_connects_normally(server) -> None:
    port = server.socket.getsockname()[1]
    transport = WebSocketTransport("127.0.0.1", robot_id=_SERVER_ROBOT_ID, port=port)
    try:
        auth = transport.connect(timeout=5.0)
        assert auth["robot_id"] == _SERVER_ROBOT_ID
    finally:
        transport.close()


def test_wrong_robot_id_is_refused_with_a_clear_error(server) -> None:
    port = server.socket.getsockname()[1]
    transport = WebSocketTransport("127.0.0.1", robot_id="WRONG_ID", port=port)
    with pytest.raises(BonicConnectionError) as excinfo:
        transport.connect(timeout=5.0)
    message = str(excinfo.value)
    assert str(protocol.CLOSE_CODE_WRONG_ROBOT) in message
    assert "WRONG_ID" in message


# --- which video path the transport picks --------------------------------
#
# The choice is not a preference: a `run_code` sandbox is `bwrap
# --unshare-net`, so a WebRTC peer there has no route to robot_app for
# signaling or media and no ICE candidate pair that can connect. Getting this
# branch wrong is silent — video simply never arrives — so it is pinned here.
#
# Both links' `start()` is stubbed out: one negotiates a real peer and the
# other needs OpenCV, and neither is what the branch under test decides. The
# link classes themselves import cleanly without aiortc or cv2 (both are
# imported lazily, inside `start`), so this runs on a bare install.


def _camera_link_for(transport, monkeypatch) -> object:
    from bonicos.transports._camera_link import NativeCameraLink
    from bonicos.transports._camera_snapshot import SnapshotCameraLink

    monkeypatch.setattr(NativeCameraLink, "start", lambda self, *a, **k: None)
    monkeypatch.setattr(SnapshotCameraLink, "start", lambda self, *a, **k: None)
    transport.start_camera(["face"])
    return transport._camera


def test_unix_socket_transport_pulls_frames_over_the_protocol(monkeypatch) -> None:
    from bonicos.transports._camera_snapshot import SnapshotCameraLink

    tx = WebSocketTransport("localhost", uds="/run/bonic/robot_app.sock")
    assert isinstance(_camera_link_for(tx, monkeypatch), SnapshotCameraLink)


def test_tcp_transport_still_uses_the_webrtc_peer(monkeypatch) -> None:
    from bonicos.transports._camera_link import NativeCameraLink

    tx = WebSocketTransport("192.168.1.50")
    assert isinstance(_camera_link_for(tx, monkeypatch), NativeCameraLink)


def test_the_snapshot_link_is_handed_the_transport_it_must_talk_over(
    monkeypatch,
) -> None:
    """It sends its own requests, so it needs the transport itself — not a
    host and port, which is what the WebRTC link takes and what a
    copy-paste of that line would leave behind."""
    tx = WebSocketTransport("localhost", uds="/run/bonic/robot_app.sock")
    assert _camera_link_for(tx, monkeypatch)._transport is tx


# --- ack latency ---------------------------------------------------------


def test_an_ack_arriving_during_the_wait_setup_is_not_missed() -> None:
    """The ack must be seen at once even if it lands in the instant between
    the caller finding it absent and the caller starting to wait.

    That instant is the whole bug. The previous implementation kept `_acks`
    under a plain lock and signalled arrivals with a separate Event, pulsed
    `set()` then immediately `clear()`. A pulse fired while the caller was in
    that gap left nothing behind: the Event was clear again by the time the
    caller waited on it, so it slept out a full 100 ms slice with the ack
    already in the dict.

    The gap is microseconds on an idle laptop, which is why the rest of this
    suite never caught it — the in-process server always replies before the
    caller waits at all, so the wait is skipped outright. On an A2 running
    the nav stack it is routinely lost instead: the rx thread is parsing a
    ~130 KB frame and the GIL is contended by the rclpy executor, so the
    caller is regularly descheduled inside the gap. Measured there, ~100 ms
    on every acked command.

    Reproduced by publishing at exactly that point, which means reaching into
    where `wait_for_ack` looks at `_disconnected` — deliberately coupled to
    the implementation, because the ordering IS what is under test. A
    Condition closes the gap by construction: the caller holds the lock
    across both the check and the start of the wait, so the publisher cannot
    get in between them at all.
    """
    tx = WebSocketTransport("127.0.0.1")
    published = threading.Event()
    really_disconnected = tx._disconnected.is_set

    def publish() -> None:
        tx._dispatch({"type": "ack", "id": 1, "ok": True})

    def hook() -> bool:
        if not published.is_set():
            published.set()
            threading.Thread(target=publish, daemon=True).start()
            time.sleep(0.01)   # give the publisher every chance to get in
        return really_disconnected()

    tx._disconnected.is_set = hook  # type: ignore[method-assign]

    started = time.monotonic()
    assert tx.wait_for_ack(1, timeout=5.0)["ok"] is True
    elapsed = time.monotonic() - started
    # A missed pulse costs a ~100 ms slice. 50 ms separates the two without
    # being tight enough to flake on the 10 ms the hook itself sleeps.
    assert elapsed < 0.05, f"ack took {elapsed * 1000:.0f} ms to be noticed"


def test_a_disconnect_wakes_a_waiter_immediately(server) -> None:
    """The close path must notify under the same lock too, or a waiter sleeps
    to its full timeout after the socket is already gone."""
    tx = WebSocketTransport("127.0.0.1", port=server.socket.getsockname()[1])
    tx.connect()

    # `drive` is the one command this server deliberately never replies to,
    # matching a robot: it is high-rate and unacked by protocol.
    cmd_id = tx.send({"type": "drive", "linear_x": 0.0})
    threading.Timer(0.1, tx.close).start()

    started = time.monotonic()
    with pytest.raises(RobotDisconnected):
        tx.wait_for_ack(cmd_id, timeout=30.0)
    assert time.monotonic() - started < 5.0
