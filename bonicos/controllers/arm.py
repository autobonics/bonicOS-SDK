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

#: Gripper end stops, from ``protocol.GRIPPER_RANGE_DEG`` — the narrowest
#: range valid on every series.
#:
#: These were -90/+90 until 2026-09-04, carried over from the old BLE SDK's
#: ``ServoConstants``. That range does not exist on ANY robot: the gripper
#: travels -45..60 on A/S and -60..60 on M (bonicOS-firmware
#: ``bonicbot_actuator_naming.md``, matching both URDFs). Commanding 90 got
#: clamped to 60 by the URDF limit, and then ``_wait_for_convergence`` sat
#: waiting for a joint to reach an angle it physically cannot — so
#: ``open_grippers()`` and ``close_grippers()`` returned False after a 5s
#: timeout on every robot, having actually worked.
#:
#: Polarity (positive = open) is unchanged and still unverified against
#: hardware; only the magnitudes are fixed here.
GRIPPER_OPEN_DEG = protocol.GRIPPER_RANGE_DEG[1]  # 60.0
GRIPPER_CLOSE_DEG = protocol.GRIPPER_RANGE_DEG[0]  # -45.0

#: Well inside neck yaw's +/-90 on every series, so these need no clamping.
#: Positive is RIGHT, negative is LEFT — verified against hardware
#: 2026-09-15. (Not the ROS/REP-103 right-hand-rule reading of the URDF's
#: `axis xyz="0 0 1"`, which would put positive on the left; the physical
#: robot disagrees with that reading, and the physical robot wins.)
NECK_LEFT_DEG = -45.0
NECK_RIGHT_DEG = 45.0
NECK_CENTER_DEG = 0.0


class ArmController(ControllerBase):
    #: Convergence tolerance for `wait=True` servo commands — the exact
    #: 0.15 rad criterion vetted end-to-end against real hardware/sim
    #: (bonicOS-m1-ros/multiTestReport.md §4), expressed in degrees since
    #: this API boundary is degrees.
    CONVERGENCE_TOLERANCE_DEG = 8.6  # math.degrees(0.15)

    #: Pacing while polling for convergence.
    _UPDATE_POLL_TIMEOUT_S = 0.2

    #: When the caller doesn't pass an explicit `timeout`, pad generously
    #: above the nominal `duration` rather than assuming wall-clock time
    #: matches it (proven necessary against Gazebo's
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
        """Return every actuator this robot has to 0 degrees.

        Scoped to the joints the robot actually reports rather than all 18 in
        the registry — on A2 that is 7, and naming the other 11 just to have
        the server drop them makes a clean call look like a partial failure.
        Falls back to the full registry before the first ``joint_states``
        frame, where the SDK has nothing better to go on.
        """
        observed = self.get_servo_angles()
        targets = observed or {joint.value: 0.0 for joint in ServoID}
        return self._send_servo_command({key: 0.0 for key in targets})

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
        """Expand ``angles`` so every touched group carries its full joint
        set **as this robot actually reports it**, holding any joint the
        caller didn't specify at its current measured position.

        Required for correctness, not just completeness: verified against
        the real M1 sim (2026-08-04, cross-checked against an independent
        60-iteration ROS-level stress test in bonicOS-m1-ros) that
        ``left_arm``/``right_arm``/``head`` — all backed by
        ``JointTrajectoryController``/``JointGroupPositionController`` —
        silently ignore a command that omits any joint they claim. Without
        this, convenience methods like ``move_left_arm(shoulder, elbow)``
        (2 of 7 left_arm joints on M) are a no-op.

        **Filled from ``joint_states``, not from the table.** ``JOINT_GROUPS``
        is the 18-actuator M fitment; A fits 7 of those and S 14, and per-robot
        NVS config means fitment isn't even fixed per series (see
        ``protocol.JOINT_GROUPS``' note). Filling from the table sent A2 five
        joints it does not have on every ``move_left_arm`` — harmless on the
        wire (the server drops them and reports them as ``unsupported``) but
        the SDK then waited for them to converge, and a joint that does not
        exist never reports a position. ``move_left_arm``, ``set_neck`` and
        ``reset_servos`` therefore returned False after a 5s timeout on A2
        while the arm moved correctly.

        Filling only what the robot reports also removes the old ``0.0``
        default, which was its own hazard: robot_app refuses to guess 0.0 for
        a joint it has never seen precisely because an unrequested joint
        snapping to zero is dangerous on hardware, and the SDK was walking
        past that guard by sending an explicit 0.0.

        Before the first ``joint_states`` frame arrives there is nothing to
        fill from, so the command goes out as the caller wrote it and the
        server decides — it has its own last-sample fill and will answer
        ``failed`` rather than move a joint blind.

        Keys that aren't part of any known group (typos) pass through
        unchanged for the server's normal ``unknown`` handling.
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
                if key not in filled and key in current:
                    filled[key] = current[key]
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

        # Exclude any key the server said it did not drive (PROTOCOL.md §5.4):
        #   `unknown`     — not a registry joint at all (a typo).
        #   `unsupported` — a real registry joint this robot does not fit.
        # Neither reached a servo, so waiting for either to "converge" would
        # spuriously time out the whole call even though every joint the robot
        # does have got there fine. `unsupported` is the A2 case: the registry
        # is shared across series and names 18 actuators, A2 fits 7.
        #
        # An older robot_app reports neither for an absent joint, which is why
        # `_fill_group` no longer sends them in the first place — this is the
        # second line of defence, for a joint the CALLER named explicitly.
        not_driven = set(result.get("unknown", [])) | set(result.get("unsupported", []))
        targets = {key: angle for key, angle in angles.items() if key not in not_driven}
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
        commanded ``duration`` — proven necessary, not just stylistic:
        a fixed sleep keyed to duration produced
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
