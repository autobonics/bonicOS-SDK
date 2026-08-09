"""System (API.md §10): health, wifi, updates, speech, on-device LLM."""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from .. import protocol
from ._base import ControllerBase


class SystemController(ControllerBase):
    def health(self) -> dict:
        return self._command({"type": protocol.CMD_HEALTH})

    # --- base session (the ROS stack under mapping/navigation) -------------

    def restart_base_session(self, timeout: float = 120.0) -> bool:
        """Full cycle: nav session down, base stack down, base stack up, nav
        session back. The operator-facing recovery action for a wedged robot
        — start/stop aren't exposed separately because a bare stop leaves a
        robot that can only be revived over SSH.

        Slow (a cold Gazebo start alone is ~25s, plus nav teardown/AMCL
        reseed on top — worst case over a minute), hence the long default
        timeout; a WebRTC video peer will drop partway through since the
        restart takes the camera topics down with it too. Refused (``False``)
        while the robot is under manual drive or running a navigation goal —
        cancel/stop that first. Feature-gated on ``session_control``.
        """
        self._require_feature("session_control")
        result = self._command(
            {"type": protocol.CMD_RESTART_BASE_SESSION}, timeout=timeout
        )
        return bool(result.get("ok", False))

    def get_session_status(self) -> Dict[str, Any]:
        """Synchronous, ungated point-in-time read of the full session state:
        ``{"base": {...}, "nav": {...}, "health": {...}}`` — the same
        underlying state the ``base_session``/``session_health`` telemetry
        events push on change, in one round trip without waiting for a push.
        """
        return self._command({"type": protocol.CMD_GET_SESSION_STATUS})

    def get_base_session(self) -> Optional[Dict[str, Any]]:
        """Latest cached ``base_session`` telemetry: ``{"running", "owned",
        "transitioning", "error"}``, or ``None`` before the first frame
        arrives (e.g. right after connecting)."""
        return self._latest(protocol.EVENT_BASE_SESSION)

    def get_session_health(self) -> Optional[Dict[str, Any]]:
        """Latest cached ``session_health`` telemetry: ``{"ok", "base",
        "nav", "issues"}`` — ``issues`` names the mechanism (e.g.
        ``"amcl_not_running"``), not just a boolean. ``None`` before the
        first frame arrives. Pushed only on change, so a robot that's been
        healthy the whole session may still be ``None`` right after connect;
        use ``get_session_status()`` for a guaranteed-fresh read.
        """
        return self._latest(protocol.EVENT_SESSION_HEALTH)

    def reconfig_wifi(self, ssid: str, password: str) -> bool:
        result = self._command(
            {"type": protocol.CMD_RECONFIG_WIFI, "ssid": ssid, "password": password}
        )
        return bool(result.get("ok", False))

    def trigger_update(self) -> bool:
        result = self._command({"type": protocol.CMD_TRIGGER_UPDATE})
        return bool(result.get("ok", False))

    def speak(self, text: str, voice: Optional[str] = None) -> bool:
        """Say ``text``. The robot decides *where* it's produced (PROTOCOL §5.6)
        — the caller never picks a route."""
        payload: Dict[str, object] = {"type": protocol.CMD_SPEAK, "text": text}
        if voice is not None:
            payload["voice"] = voice
        result = self._command(payload)
        return bool(result.get("ok", False))

    def ask_llm(self, prompt: str, model: Optional[str] = None, timeout: float = 60.0) -> str:
        """On-device LLM. **Display only** — never executed as a command.

        Blocks and returns the full text; tokens stream internally
        (PROTOCOL.md §5.7 ``llm_query`` -> a stream of ``llm_token`` events
        carrying this command's id).
        """
        payload: Dict[str, object] = {"type": protocol.CMD_LLM_QUERY, "prompt": prompt}
        if model is not None:
            payload["model"] = model
        cmd_id = self._send(payload)

        drain = getattr(self._transport, "drain_events", None)
        chunks = []
        deadline = time.monotonic() + timeout
        done = False
        last_seen = None  # fallback path only: dedupe the last-value cache
        while not done:
            if drain is not None:
                for event in drain(protocol.EVENT_LLM_TOKEN):
                    if event.get("id") != cmd_id:
                        continue
                    chunks.append(event.get("token", ""))
                    if event.get("done"):
                        done = True
            else:
                # Transport has no per-message event log (e.g. webrtc/sim) —
                # fall back to the last-value cache; a fast token burst may
                # coalesce and drop intermediate chunks.
                event = self._latest(protocol.EVENT_LLM_TOKEN)
                if event is not None and event is not last_seen and event.get("id") == cmd_id:
                    last_seen = event
                    chunks.append(event.get("token", ""))
                    if event.get("done"):
                        done = True

            if done:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._transport.wait_for_update(min(remaining, 1.0))

        return "".join(chunks)
