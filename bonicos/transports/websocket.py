"""The transport — plain Python, background receive thread.

This is the only way the SDK reaches a robot, whether it runs on a
developer's laptop, inside the on-robot runner (``host="127.0.0.1"``), or
anywhere else on the robot's LAN. Video is the one exception and it is
invisible: :meth:`WebSocketTransport.start_camera` negotiates a WebRTC peer
behind the scenes (``_camera_link.py``), because media tracks are the only
way video leaves the robot.

A background thread iterates the socket and rebinds the latest value per
event under the GIL, plus a couple of ``threading.Event``s to let
synchronous callers block without spinning — which is what keeps the public
API blocking and ``async``-free.

``websockets`` is imported lazily, inside :meth:`WebSocketTransport.connect`
only — never at module import time. It is a base dependency, so it is
always installed; the lazy import is about keeping ``import bonicos``
cheap, not about it being absent.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.parse
from typing import Any, Dict, Optional

from .. import protocol
from ..exceptions import ConnectionError as BonicConnectionError
from ..exceptions import RobotDisconnected
from . import base


class WebSocketTransport:
    """Talks the ``bonicos`` wire protocol over a plain WebSocket.

    Connects to ``ws://<host>:<port>/ws`` (PROTOCOL.md §1), or
    ``ws://<host>:<port>/ws?robotId=<robot_id>`` when ``robot_id`` is given —
    the same URL shape regardless of which process hosts it
    (`bonicOS-robot-app` on Pro, the Flutter tablet app on Lite).
    """

    def __init__(
        self,
        host: str,
        *,
        robot_id: Optional[str] = None,
        port: int = 8080,
        token: Optional[str] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._robot_id = robot_id
        self._token = token or ""

        self._ws: Any = None
        self._connection_closed_exc: Any = Exception
        self._rx_thread: Optional[threading.Thread] = None

        self._id_counter = 0
        self._id_lock = threading.Lock()

        self._latest: Dict[str, dict] = {}
        self._event_log: Dict[str, list] = {}
        self._event_log_lock = threading.Lock()
        self._update_event = threading.Event()
        # Monotonic count of telemetry dispatches, so wait_for_update() can
        # poll "did anything land since I started" instead of relying solely
        # on the Event pulse — see wait_for_update's docstring for why.
        self._update_seq = 0

        self._acks: Dict[int, dict] = {}
        self._acks_lock = threading.Lock()
        self._acks_event = threading.Event()

        self._auth_event = threading.Event()
        self._auth_result: Dict[str, Any] = {}
        self._auth_error: Optional[str] = None

        self._closed_code: Optional[int] = None
        self._disconnected = threading.Event()

        self._camera: Any = None  # NativeCameraLink, created on first use

    # --- Transport protocol ------------------------------------------------

    #: Video rides a WebRTC peer this transport brings up on demand
    #: (start_camera) — commands/telemetry stay on the WebSocket.
    supports_camera = True

    def connect(self, timeout: float = 10.0) -> dict:
        import websockets.sync.client as ws_sync_client
        from websockets.exceptions import ConnectionClosed

        self._connection_closed_exc = ConnectionClosed

        url = self._build_url()
        try:
            self._ws = ws_sync_client.connect(url, open_timeout=timeout)
        except Exception as exc:  # noqa: BLE001 - surfaced as our own type
            raise BonicConnectionError(f"failed to connect to {url}: {exc}") from exc

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        try:
            self.send(
                {
                    "type": protocol.TYPE_AUTH,
                    "token": self._token,
                    "protocol_version": protocol.PROTOCOL_VERSION,
                }
            )
        except RobotDisconnected:
            # The server closed the socket before we could even send auth
            # (e.g. an immediate robotId-mismatch close). The rx thread will
            # shortly notice the same closure and pulse _auth_event, so fall
            # through to the same close-code handling below instead of
            # raising here.
            pass

        if not self._auth_event.wait(timeout):
            self.close()
            raise BonicConnectionError("timed out waiting for auth_result")

        if not self._auth_result:
            # _auth_event is also pulsed by _rx_loop's finally block on any
            # disconnect, so reaching here without a timeout can still mean
            # the server closed the socket before ever sending auth_result
            # (e.g. an immediate robotId-mismatch close) rather than a
            # successful handshake — check that before trusting the empty
            # result.
            code = self._closed_code
            self.close()
            if code == protocol.CLOSE_CODE_WRONG_ROBOT:
                raise BonicConnectionError(
                    f"server closed the connection ({protocol.CLOSE_CODE_WRONG_ROBOT}):"
                    f" robotId {self._robot_id!r} does not match this robot"
                )
            raise BonicConnectionError(
                "connection closed before auth completed"
                + (f" (code {code})" if code is not None else "")
            )

        if self._auth_error is not None:
            raise BonicConnectionError(self._auth_error)

        return dict(self._auth_result)

    def send(self, msg: dict) -> int:
        with self._id_lock:
            self._id_counter += 1
            cmd_id = self._id_counter

        payload = dict(msg)
        if payload.get("type") not in protocol.UNACKED_COMMANDS:
            payload["id"] = cmd_id

        if self._ws is None:
            raise RobotDisconnected("not connected")
        try:
            self._ws.send(json.dumps(payload))
        except self._connection_closed_exc as exc:
            raise RobotDisconnected("connection closed") from exc

        return cmd_id

    def read_telemetry(self) -> dict:
        return dict(self._latest)

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        """Block until the next telemetry frame arrives, or time out.

        Polls ``_update_seq`` in short slices rather than a single
        ``Event.wait(timeout)``: the rx thread pulses ``_update_event``
        (``set()`` immediately followed by ``clear()``) per message, which a
        caller that hasn't reached its ``wait()`` yet can miss entirely — a
        real missed-wakeup race, not just a theoretical one (reproduced:
        ``auth_result`` immediately followed by a telemetry push landed its
        pulse before the caller's first ``wait_for_update()`` call, which
        then blocked for the full timeout despite the data already sitting
        in ``_latest``). Re-checking the counter every slice caps a missed
        pulse's cost at one slice instead of the whole timeout.
        """
        seen = self._update_seq
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or self._disconnected.is_set():
                return False
            self._update_event.wait(min(remaining, 0.1))
            if self._update_seq != seen:
                return True

    def wait_for_ack(self, cmd_id: int, timeout: float = 5.0) -> dict:
        deadline = time.monotonic() + timeout
        while True:
            with self._acks_lock:
                if cmd_id in self._acks:
                    return self._acks.pop(cmd_id)
            if self._disconnected.is_set():
                raise RobotDisconnected(
                    f"connection closed while waiting for ack of command {cmd_id}"
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BonicConnectionError(
                    f"timed out waiting for ack of command {cmd_id}"
                )
            self._acks_event.wait(min(remaining, 0.1))

    def start_camera(self, cameras: list) -> None:
        """Bring up the behind-the-scenes WebRTC video peer (idempotent).

        Commands and telemetry keep flowing over this WebSocket; only video
        rides the WebRTC peer, transparently to the caller.
        """
        if self._camera is None:
            from ._camera_link import NativeCameraLink

            self._camera = NativeCameraLink(self._host, self._port)
        self._camera.start(cameras)

    def stop_camera(self) -> None:
        if self._camera is not None:
            self._camera.stop()
            self._camera = None

    def read_frame(self, camera: Optional[str] = None) -> Optional[base.Frame]:
        if self._camera is None:
            return None
        return self._camera.read_frame(camera)

    def drain_events(self, event_type: str) -> list:
        """Every buffered message of ``event_type`` since the last drain.

        Not part of the formal :class:`~bonicos.transports.base.Transport`
        protocol (:mod:`bonicos.transports.base` lists no such method) — an extra a
        controller can reach for when it needs every message of a
        fast-moving ``ASYNC_EVENTS`` type rather than just the last-value
        snapshot ``read_telemetry()`` gives (e.g. ``system.ask_llm``'s
        ``llm_token`` chunks). Mirrors the ``self._events.append(msg)``
        branch in this module's ``_rx_loop``.
        """
        with self._event_log_lock:
            events = self._event_log.get(event_type, [])
            self._event_log[event_type] = []
        return events

    def close(self) -> None:
        self.stop_camera()
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:  # noqa: BLE001 - best-effort on the way out
                pass
        if (
            self._rx_thread is not None
            and self._rx_thread is not threading.current_thread()
        ):
            self._rx_thread.join(timeout=2.0)
        self._ws = None

    # --- background receive thread -----------------------------------------

    def _build_url(self) -> str:
        if self._robot_id is None:
            return f"ws://{self._host}:{self._port}/ws"
        query = urllib.parse.urlencode({"robotId": self._robot_id})
        return f"ws://{self._host}:{self._port}/ws?{query}"

    def _rx_loop(self) -> None:
        try:
            for raw in self._ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                self._dispatch(msg)
        except self._connection_closed_exc as exc:
            self._closed_code = getattr(getattr(exc, "rcvd", None), "code", None)
        finally:
            self._disconnected.set()
            self._auth_event.set()  # unblock connect() if still waiting
            self._acks_event.set()
            self._acks_event.clear()
            self._update_event.set()
            self._update_event.clear()

    def _dispatch(self, msg: dict) -> None:
        msg_type = msg.get("type")

        if msg_type == protocol.TYPE_AUTH_RESULT:
            self._auth_result = msg
            if not msg.get("ok", True):
                self._auth_error = msg.get("error", "authentication failed")
            self._auth_event.set()
            return

        if msg_type in (protocol.TYPE_ACK, protocol.TYPE_ERROR):
            cmd_id = msg.get("id")
            if cmd_id is not None:
                with self._acks_lock:
                    self._acks[cmd_id] = msg
                self._acks_event.set()
                self._acks_event.clear()
            return

        if msg_type in protocol.TELEMETRY_EVENTS or msg_type in protocol.ASYNC_EVENTS:
            self._latest[msg_type] = msg  # atomic rebind under the GIL
            if msg_type in protocol.ASYNC_EVENTS:
                with self._event_log_lock:
                    self._event_log.setdefault(msg_type, []).append(msg)
            self._update_seq += 1
            self._update_event.set()
            self._update_event.clear()
            return

        # Unknown message type: ignore rather than fail the rx thread, so a
        # forward-compatible server addition never breaks an older SDK.
