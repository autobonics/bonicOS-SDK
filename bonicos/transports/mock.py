"""A deterministic, hardware-free transport double for unit tests.

No network, no ROS, no browser. Telemetry is set directly by the test;
acks are scripted per command type; every ``send()`` call is recorded so
tests can assert on the exact payload a controller built.
"""

from __future__ import annotations

import itertools
import threading
import time
from typing import Any, Dict, List, Optional

from .. import protocol


class MockTransport:
    """Test double implementing the :class:`bonicos.transports.base.Transport`
    protocol structurally (see ``base.py`` — no explicit inheritance needed).
    """

    def __init__(self) -> None:
        self._id_counter = itertools.count(1)
        self._connected = False
        self._telemetry: Dict[str, dict] = {}
        self._event_log: Dict[str, List[dict]] = {}
        self._update_event = threading.Event()
        # See websocket.py's wait_for_update docstring: set()+clear() alone
        # is a missed-wakeup race against a caller that hasn't reached
        # wait() yet — this counter closes that gap. Real here too, not just
        # theoretical: push_event/set_telemetry run on whatever thread the
        # test uses (e.g. a background pusher in test_system.py's ask_llm
        # streaming test), racing a consumer thread's wait_for_update().
        self._update_seq = 0
        self.sent: List[dict] = []
        self._default_acks: Dict[str, dict] = {}
        self._scripted_acks: Dict[int, dict] = {}
        self._auth_result = {
            "robot_id": "MOCK_001",
            "series": "M",
            "features": {},
        }

    # --- test-side configuration ---------------------------------------

    def set_auth_result(self, **fields: Any) -> None:
        self._auth_result = {**self._auth_result, **fields}

    def set_telemetry(self, event: str, payload: dict) -> None:
        """Set the latest value for a telemetry event and wake any waiter."""
        self._telemetry[event] = {"type": event, **payload}
        self._update_seq += 1
        self._update_event.set()
        self._update_event.clear()

    def script_ack(self, command_type: str, result: Optional[dict] = None) -> None:
        """Every future ``send()`` of this command type acks with ``result``."""
        self._default_acks[command_type] = result or {}

    def script_ack_for_id(self, cmd_id: int, result: dict) -> None:
        self._scripted_acks[cmd_id] = result

    def push_event(self, event: str, payload: dict) -> None:
        """Push an async event (e.g. ``nav_status``, ``llm_token``).

        Updates the last-value cache (for ``read_telemetry()``/
        ``get_nav_status()``-style getters) *and* appends to the per-type
        event log so a burst of same-type messages — e.g. streaming
        ``llm_token`` chunks — isn't lost to last-value-wins coalescing.
        Mirrors the ``self._events.append(msg)`` branch in
        the real transport's ``_rx_loop``.
        """
        msg = {"type": event, **payload}
        self._telemetry[event] = msg
        self._event_log.setdefault(event, []).append(msg)
        self._update_seq += 1
        self._update_event.set()
        self._update_event.clear()

    # --- Transport protocol ----------------------------------------------

    def connect(self, timeout: float = 10.0) -> dict:
        self._connected = True
        return dict(self._auth_result)

    def send(self, msg: dict) -> int:
        cmd_id = next(self._id_counter)
        payload = dict(msg)
        if payload.get("type") not in protocol.UNACKED_COMMANDS:
            payload["id"] = cmd_id
        self.sent.append(payload)
        return cmd_id

    def read_telemetry(self) -> dict:
        return dict(self._telemetry)

    def wait_for_update(self, timeout: float = 1.0) -> bool:
        seen = self._update_seq
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._update_event.wait(min(remaining, 0.1))
            if self._update_seq != seen:
                return True

    def wait_for_ack(self, cmd_id: int, timeout: float = 5.0) -> dict:
        if cmd_id in self._scripted_acks:
            return {
                "type": protocol.TYPE_ACK,
                "id": cmd_id,
                **self._scripted_acks[cmd_id],
            }
        # Find the command type this id was sent for.
        command_type: Optional[str] = None
        for i, msg in enumerate(self.sent, start=1):
            if i == cmd_id:
                command_type = msg.get("type")
                break
        result = (
            self._default_acks.get(command_type, {}) if command_type is not None else {}
        )
        return {"type": protocol.TYPE_ACK, "id": cmd_id, **result}

    #: The offline mock has no video path at all.
    supports_camera = False

    def start_camera(self, cameras: list) -> None:
        from ..exceptions import CameraUnavailable

        raise CameraUnavailable("the mock transport has no camera")

    def stop_camera(self) -> None:
        return None

    def read_frame(self, camera: Optional[str] = None) -> None:
        return None

    def close(self) -> None:
        self._connected = False

    # --- extra: async-event draining (not part of the formal Transport ---
    # --- protocol — used by controllers that need every message of a    ---
    # --- fast-moving event type, e.g. system.ask_llm's llm_token chunks) --

    def drain_events(self, event_type: str) -> List[dict]:
        events = self._event_log.get(event_type, [])
        self._event_log[event_type] = []
        return events
