"""Shared plumbing every feature controller delegates to.

Not part of the public API (see API.md for the public controller modules; this
is the implementation detail that keeps ack/error handling from being
duplicated seven times).

**No capability checks live here, or anywhere else in the SDK** (PROTOCOL.md
§3.1). Controllers send what they are asked to send; a robot that cannot
perform a command refuses it — an ``error``, or an ``ack`` with ``ok: false``
— and ``_command`` raises either as ``CommandError`` carrying the server's own
explanation. Do not reintroduce a local gate — the server is the only party
that knows its own hardware.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional

from .. import protocol
from ..exceptions import CommandError
from ..transports.base import Transport

if TYPE_CHECKING:
    from ..robot import BonicBot


class ControllerBase:
    def __init__(self, robot: "BonicBot") -> None:
        self._robot = robot

    @property
    def _transport(self) -> Transport:
        return self._robot._transport

    def _send(self, msg: Dict[str, Any]) -> int:
        """Enqueue a command; returns the id used to correlate its ack."""
        return self._transport.send(msg)

    def _command(self, msg: Dict[str, Any], *, timeout: float = 5.0) -> dict:
        """Send a command and block for its ack, raising on any refusal.

        Two reply shapes are refusals (PROTOCOL.md §2): an ``error``, and an
        ``ack`` with ``ok: false``. Both raise ``CommandError`` here, so a
        result returned from this method always succeeded. An ack with no
        ``ok`` field (``nav_goal``'s ``{goal_id}``, ``list_maps``) is success;
        only an explicit ``False`` is a refusal.
        """
        cmd_id = self._send(msg)
        result = self._transport.wait_for_ack(cmd_id, timeout)
        command = msg.get("type", "?")
        if result.get("type") == protocol.TYPE_ERROR:
            raise CommandError(
                command, result.get("error") or "unknown error", result
            )
        if result.get("ok") is False:
            raise CommandError(command, self._refusal_reason(result), result)
        return result

    @staticmethod
    def _refusal_reason(result: dict) -> str:
        """The reason carried by an ``ok: false`` ack: ``error``, else
        ``detail``, else a generic sentence."""
        for key in ("error", "detail"):
            reason = result.get(key)
            if reason:
                return str(reason)
        return "the robot refused this without saying why — check its log"

    def _latest(self, event: str) -> Optional[dict]:
        """Latest cached telemetry for ``event``, or None.

        None is ambiguous on a robot that can never produce ``event`` — it
        reads the same as "nothing has arrived yet". That is an accepted
        consequence of not modelling capability (PROTOCOL.md §3.1).
        """
        return self._transport.read_telemetry().get(event)
