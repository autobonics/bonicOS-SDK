"""Simulation transport — the Three.js digital twin, same bridge as WebRTC.

Per ``bonic-architecture.md`` §10: "The simulation backend writes into the
identical buffers... Python cannot tell the difference." So this is a thin
wrapper around :class:`~bonicos.transports.webrtc.WebRTCTransport`, not a
reimplementation — it only differs in which host-provided global it binds
to, so a real robot's WebRTC bridge and the digital twin's sim bridge can
run simultaneously without the SDK layer branching on which is which.
"""

from __future__ import annotations

from .webrtc import WebRTCTransport

#: The host is expected to expose the digital twin's buffers under a
#: separate global from the real robot's WebRTC bridge (see
#: ``webrtc.DEFAULT_BRIDGE_GLOBAL``), so both can be bound at once
#: (ARCHITECTURE.md §7 / bonic-architecture.md §10: "run both backends
#: simultaneously").
DEFAULT_SIM_BRIDGE_GLOBAL = "__bonicosSimBridge__"


class SimTransport(WebRTCTransport):
    """Drives the Three.js digital twin through the same SAB mechanism."""

    def __init__(self, *, bridge_global: str = DEFAULT_SIM_BRIDGE_GLOBAL) -> None:
        super().__init__(bridge_global=bridge_global)
