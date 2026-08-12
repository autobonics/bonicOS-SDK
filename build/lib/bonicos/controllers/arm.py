"""Arms, grippers & neck (API.md §5) — built on ``servo_command``.

Angles are **degrees** at this API boundary (adopted default #2), converted
to radians here before the wire send (PROTOCOL.md §5.4: ``servo_command``
carries radians).
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

from .. import protocol
from ..enums import ServoID
from ._base import ControllerBase

#: Placeholder open/closed angles pending real hardware tuning — the servo
#: range (-90..90) is taken from the old BLE SDK's ``ServoConstants``
#: (``Bonicbot-SDKs/bonicbot/bonicbot/controllers/models.py``), but which
#: end is physically "open" vs "closed" is hardware-specific.
GRIPPER_OPEN_DEG = 90.0
GRIPPER_CLOSE_DEG = -90.0

NECK_LEFT_DEG = 45.0
NECK_RIGHT_DEG = -45.0
NECK_CENTER_DEG = 0.0


class ArmController(ControllerBase):
    #: Convergence tolerance for `wait=True` servo commands — the exact
    #: 0.15 rad criterion vetted end-to-end against real hardware/sim
    #: (bonicOS-m1-ros/multiTestReport.md §4), expressed in degrees since
    #: this API boundary is degrees. dev/ARCHITECTURE.md §4a.
    CONVERGENCE_TOLERANCE_DEG = 8.6  # math.degrees(0.15)

    #: Pacing while polling for convergence.
    _UPDATE_POLL_TIMEOUT_S = 0.2

    #: When the caller doesn't pass an explicit `timeout`, pad generously
    #: above the nominal `duration` rather than assuming wall-clock time
    #: matches it (dev/ARCHITECTURE.md §4a — proven necessary against Gazebo's
    #: sub-1.0 RTF; harmless slack on real hardware, which finishes sooner).
    _DEFAULT_TIMEOUT_MIN_S = 5.0
    _DEFAULT_TIMEOUT_DURATION_MULTIPLIER = 3.0

    def set_servos(
        self,
        angles: Dict[str, float],
        duration: float = 1.0,
        *,
        wait: bool = True,
        timeout: Optional[float] = None,
    ) -> bool:
        return self._send_servo_command(angles, duration, wait=wait, timeout=timeout)

    def move_left_arm(
        self,
        shoulder: float,
        elbow: float,
        wait: bool = True,
        duration: float = 1.0,
        timeout: Optional[float] = None,
    ) -> bool:
        return self._send_servo_command(
            {
                ServoID.LEFT_SHOULDER_PITCH.value: shoulder,
                ServoID.LEFT_ELBOW.value: elbow,
            },
            duration,
            wait=wait,
            timeout=timeout,
        )

    def move_right_arm(
        self,
        shoulder: float,
        elbow: float,
        wait: bool = True,
        duration: float = 1.0,
        timeout: Optional[float] = None,
    ) -> bool:
        return self._send_servo_command(
            {
                ServoID.RIGHT_SHOULDER_PITCH.value: shoulder,
                ServoID.RIGHT_ELBOW.value: elbow,
            },
            duration,
            wait=wait,
            timeout=timeout,
        )

    def set_grippers(self, left: float, right: float) -> bool:
        return self._send_servo_command(
            {ServoID.LEFT_GRIPPER.value: left, ServoID.RIGHT_GRIPPER.value: right}
        )

    def open_grippers(self) -> bool:
        return self.set_grippers(GRIPPER_OPEN_DEG, GRIPPER_OPEN_DEG)

    def close_grippers(self) -> bool:
        return self.set_grippers(GRIPPER_CLOSE_DEG, GRIPPER_CLOSE_DEG)

    def set_neck(self, yaw: float) -> bool:
        return self._send_servo_command({ServoID.NECK_YAW.value: yaw})

    def look_left(self) -> bool:
        return self.set_neck(NECK_LEFT_DEG)

    def look_right(self) -> bool:
        return self.set_neck(NECK_RIGHT_DEG)

    def look_center(self) -> bool:
        return self.set_neck(NECK_CENTER_DEG)

    def reset_servos(self) -> bool:
        return self._send_servo_command({joint.value: 0.0 for joint in ServoID})

    def set_single_servo(
        self,
        joint: str,
        angle: float,
        speed: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        payload: Dict[str, object] = {
            "type": protocol.CMD_SERVO_SINGLE,
            "joint": joint,
            "angle": angle,
        }
        if speed is not None:
            payload["speed"] = speed
        if acc is not None:
            payload["acc"] = acc
        result = self._command(payload)
        return bool(result.get("ok", False))

    def get_servo_angles(self) -> Dict[str, float]:
        """Latest joint positions, keyed by **registry camelCase** — the
        same keys ``set_servos``/``move_left_arm``/etc. accept — not the raw
        snake_case URDF names ``joint_states`` reports on the wire
        (PROTOCOL.md §6; ``protocol.JOINT_NAME_MAP`` is the translation).
        Only joints present in the registry *and* currently reported are
        included (e.g. wheel joints never appear here).
        """
        event = self._latest(protocol.EVENT_JOINT_STATES)
        if not event:
            return {}
        raw = dict(zip(event.get("name", []), event.get("position", [])))
        return {
            key: math.degrees(raw[joint])
            for key, joint in protocol.JOINT_NAME_MAP.items()
            if joint in raw
        }

    # --- internal ------------------------------------------------------

    def _fill_group(self, angles: Dict[str, float]) -> Dict[str, float]:
        """Expand ``angles`` so every touched group carries its FULL joint
        set, holding any joint the caller didn't specify at its current
        measured position.

        Required for correctness, not just completeness: verified against
        the real M1 sim (2026-08-04, cross-checked against an independent
        60-iteration ROS-level stress test in bonicOS-m1-ros) that
        ``left_arm``/``right_arm``/``head`` — all backed by
        ``JointTrajectoryController``/``JointGroupPositionController`` —
        silently ignore a command that omits any joint they claim. Without
        this, convenience methods like ``move_left_arm(shoulder, elbow)``
        (2 of 7 left_arm joints) are a no-op. Keys that aren't part of any
        known group (typos) pass through unchanged for the server's normal
        ``unknown`` handling.
        """
        groups_touched = {
            protocol.JOINT_GROUP_OF[key]
            for key in angles
            if key in protocol.JOINT_GROUP_OF
        }
        if not groups_touched:
            return angles
        current = self.get_servo_angles()
        filled = dict(angles)
        for group in groups_touched:
            for key in protocol.JOINT_GROUPS[group]:
                if key not in filled:
                    filled[key] = current.get(key, 0.0)
        return filled

    def _send_servo_command(
        self,
        angles: Dict[str, float],
        duration: float = 1.0,
        *,
        wait: bool = True,
        timeout: Optional[float] = None,
    ) -> bool:
        angles = self._fill_group(angles)
        payload = {
            "type": protocol.CMD_SERVO_COMMAND,
            "servos": {joint: math.radians(angle) for joint, angle in angles.items()},
            "duration": duration,
        }
        if not wait:
            self._send(payload)
            return True
        result = self._command(payload)
        if not result.get("ok", False):
            return False

        # Exclude any key the server flagged as unrecognized (PROTOCOL.md
        # §5.4 `unknown`) — it was never actually sent to a joint, so
        # waiting for it to "converge" would spuriously time out the whole
        # call even though every valid joint got there fine.
        unknown = set(result.get("unknown", []))
        targets = {key: angle for key, angle in angles.items() if key not in unknown}
        if not targets:
            return True

        effective_timeout = (
            timeout
            if timeout is not None
            else max(
                duration * self._DEFAULT_TIMEOUT_DURATION_MULTIPLIER,
                self._DEFAULT_TIMEOUT_MIN_S,
            )
        )
        return self._wait_for_convergence(targets, effective_timeout)

    def _wait_for_convergence(
        self, targets_deg: Dict[str, float], timeout: float
    ) -> bool:
        """Block until every joint in ``targets_deg`` is within
        ``CONVERGENCE_TOLERANCE_DEG`` of its target, or ``timeout`` elapses.

        Polls ``joint_states`` telemetry rather than sleeping for the
        commanded ``duration`` — proven necessary, not just stylistic
        (dev/ARCHITECTURE.md §4a): a fixed sleep keyed to duration produced
        flaky, non-reproducible false failures when tested against a
        simulated backend running below realtime; polling until the
        measured position actually matches eliminated it completely.
        """
        deadline = time.monotonic() + timeout
        while True:
            angles = self.get_servo_angles()
            if all(
                joint in angles
                and abs(angles[joint] - target) <= self.CONVERGENCE_TOLERANCE_DEG
                for joint, target in targets_deg.items()
            ):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._transport.wait_for_update(min(remaining, self._UPDATE_POLL_TIMEOUT_S))
