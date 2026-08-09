"""Shared plumbing every feature controller delegates to.

Not part of the public API (ARCHITECTURE.md §1 lists the seven public
controller modules; this is the implementation detail that keeps
ack/error/feature-gate handling from being duplicated seven times).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Dict, Optional

from .. import protocol
from ..exceptions import CommandError, FeatureUnavailable
from ..transports.base import Transport

if TYPE_CHECKING:
    from ..robot import BonicBot


class ControllerBase:
    def __init__(self, robot: "BonicBot") -> None:
        self._robot = robot

    @property
    def _transport(self) -> Transport:
        return self._robot._transport

    def _require_feature(self, feature: str) -> None:
        """Fast, local, readable failure before sending a gated command.

        The server re-gates independently (ARCHITECTURE.md §6) — this check
        exists only to fail closer to the call site with a clear message.
        """
        if not self._robot.features.get(feature, True):
            raise FeatureUnavailable(feature)

    def _send(self, msg: Dict[str, Any]) -> int:
        return self._transport.send(msg)

    def _command(self, msg: Dict[str, Any], *, timeout: float = 5.0) -> dict:
        """Send a command and block for its ack, raising on error/gating."""
        cmd_id = self._send(msg)
        result = self._transport.wait_for_ack(cmd_id, timeout)
        result_type = result.get("type")
        if result_type == protocol.TYPE_ERROR:
            raise CommandError(msg.get("type", "?"), result.get("error", "unknown error"))
        if result_type == protocol.TYPE_FEATURE_UNAVAILABLE:
            raise FeatureUnavailable(str(result.get("feature") or msg.get("type", "?")))
        return result

    def _latest(self, event: str) -> Optional[dict]:
        return self._transport.read_telemetry().get(event)
