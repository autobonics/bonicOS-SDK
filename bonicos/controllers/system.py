"""System (API.md §10): health, wifi, updates, speech."""

from __future__ import annotations

from typing import Any, Dict, Optional

from .. import protocol
from ._base import ControllerBase


class SystemController(ControllerBase):
    def health(self) -> dict:
        return self._command({"type": protocol.CMD_HEALTH})

    # --- base session (the ROS stack under mapping/navigation) -------------

    def restart_base_session(self, timeout: float = 120.0) -> bool:
        """Full cycle: nav session down, base stack down, base stack up, nav
        session back. The operator-facing recovery action for a wedged robot,
        and still the one to reach for first.

        Slow (a cold Gazebo start alone is ~25s, plus nav teardown/AMCL
        reseed on top — worst case over a minute), hence the long default
        timeout; a WebRTC video peer will drop partway through since the
        restart takes the camera topics down with it too. Refused (``False``)
        while the robot is under manual drive or running a navigation goal —
        cancel/stop that first.
        """
        result = self._command(
            {"type": protocol.CMD_RESTART_BASE_SESSION}, timeout=timeout
        )
        return bool(result.get("ok", False))

    def start_base_session(self, timeout: float = 120.0) -> bool:
        """Bring the base ROS stack up — drive, sensors, controllers, TF.

        Needed because a real robot does **not** start its stack on boot
        (robot_app's ``base_autostart`` defaults to false on hardware: bringing
        the app up must not energise servos and spin motors on its own). A
        robot that was powered on and left alone, or whose stack was stopped
        from here, has no other way back short of SSH.

        Safe to call when the stack is already up. In simulation the stack
        normally autostarts, so this is mostly a real-robot affordance.
        """
        result = self._command(
            {"type": protocol.CMD_START_BASE_SESSION}, timeout=timeout
        )
        return bool(result.get("ok", False))

    def stop_base_session(self, timeout: float = 60.0) -> bool:
        """Take the base ROS stack down — drive, sensors and TF all stop.

        The robot stops being able to move or perceive anything until
        ``start_base_session``. Carries the same guard as
        ``restart_base_session`` and is refused (``False``) while the robot is
        moving or a navigation goal is running: pulling the drive stack out
        from under a moving robot is how AMCL died on 2026-08-09, and "stop
        the stack" must never quietly also mean "abandon the goal".
        """
        result = self._command(
            {"type": protocol.CMD_STOP_BASE_SESSION}, timeout=timeout
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

    def shutdown(self, timeout: float = 15.0) -> bool:
        """Power the robot off.

        Not a stack teardown — this halts the companion computer itself, and
        where the robot's ESP lane can be reached it also cuts the power
        latch, so the robot ends up genuinely off rather than halted but still
        drawing current. Someone has to press the button to bring it back.

        Deliberately asks nothing first: by the time a program calls this it
        has decided. Idempotent — a second call while one is in flight is
        answered ``True`` rather than starting a second poweroff.

        The ack is all there is. The process answering is the one being
        halted, so nothing reports the machine actually going down; expect the
        connection to drop shortly after this returns.
        """
        result = self._command({"type": protocol.CMD_SHUTDOWN}, timeout=timeout)
        return bool(result.get("ok", False))

    def reconfig_wifi(self, ssid: str, password: str) -> bool:
        result = self._command(
            {"type": protocol.CMD_RECONFIG_WIFI, "ssid": ssid, "password": password}
        )
        return bool(result.get("ok", False))

    def update_status(self) -> Dict[str, Any]:
        """Ask the robot's host what it knows about updates, now:
        ``{"state", "phase", "percent", "version", "reported_version",
        "previous_version", "last_update", ...}``.

        ``state`` is ``installing``, ``rolling_back``, ``idle``, or
        ``unavailable`` — the last meaning there is no bonic-host to ask,
        which is normal on a bare-metal robot or a dev laptop and is not an
        update failure.

        This is the read that survives the restart an install causes: the
        cached telemetry from before the swap belongs to a connection that no
        longer exists, so after reconnecting, ask.
        """
        return self._command({"type": protocol.CMD_UPDATE_STATUS})

    def get_update_status(self) -> Optional[Dict[str, Any]]:
        """Latest cached ``update_progress`` telemetry, or ``None`` if the
        robot has said nothing about updates this session.

        Replayed on auth, so a client connecting to a robot that has just
        been updated — or rolled back — learns that immediately. Pushed only
        while an install runs; use ``update_status()`` for a fresh read.
        """
        return self._latest(protocol.EVENT_UPDATE_PROGRESS)

    def speak(self, text: str, voice: Optional[str] = None) -> bool:
        """Say ``text``. The robot decides *where* it's produced (PROTOCOL §5.6)
        — the caller never picks a route."""
        payload: Dict[str, object] = {"type": protocol.CMD_SPEAK, "text": text}
        if voice is not None:
            payload["voice"] = voice
        result = self._command(payload)
        return bool(result.get("ok", False))
