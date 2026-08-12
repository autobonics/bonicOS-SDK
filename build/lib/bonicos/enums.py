"""Enums carried over from the old BLE SDK, trimmed to the v1 core surface.

Values are the exact camelCase joint keys the ``servo_command`` wire command
expects (PROTOCOL.md §5.4) — the server's registry maps these to snake_case
URDF joints (``protocol.JOINT_NAME_MAP``). Originally ported from
``Bonicbot-SDKs/bonicbot/bonicbot/controllers/models.py`` (the old BLE-only
hardware's joint set), but that set didn't match the real M1 humanoid's
registry: it had no per-axis wrist/gripper-yaw members, and named three
joints (``rightWrist``, ``headPan``, ``headTilt``) that don't exist on M1 at
all. Corrected here to the exact 18-joint set verified end-to-end against
the real M1 registry (bonicOS-m1-ros sim, 2026-08-04) — every member's
value is a key in ``protocol.JOINT_NAME_MAP``. ``Sholder`` is corrected to
``Shoulder`` since v1 has no backward-compat requirement.
"""

from __future__ import annotations

from enum import Enum


class ServoID(str, Enum):
    """Servo identifier — a joint name in the ``servo_command`` registry."""

    # Right arm
    RIGHT_GRIPPER = "rightGripper"
    RIGHT_GRIPPER_YAW = "rightGripperYaw"
    RIGHT_WRIST_PITCH = "rightWristPitch"
    RIGHT_WRIST_YAW = "rightWristYaw"
    RIGHT_ELBOW = "rightElbow"
    RIGHT_SHOULDER_YAW = "rightShoulderYaw"
    RIGHT_SHOULDER_ROLL = "rightShoulderRoll"
    RIGHT_SHOULDER_PITCH = "rightShoulderPitch"

    # Left arm
    LEFT_GRIPPER = "leftGripper"
    LEFT_GRIPPER_YAW = "leftGripperYaw"
    LEFT_WRIST_PITCH = "leftWristPitch"
    LEFT_WRIST_YAW = "leftWristYaw"
    LEFT_ELBOW = "leftElbow"
    LEFT_SHOULDER_YAW = "leftShoulderYaw"
    LEFT_SHOULDER_ROLL = "leftShoulderRoll"
    LEFT_SHOULDER_PITCH = "leftShoulderPitch"

    # Head / neck
    NECK_YAW = "neckYaw"
    NECK_PITCH = "neckPitch"


class HeadMode(str, Enum):
    """Head expression mode for ``set_expression()`` (API.md §6, stub in v1)."""

    NORMAL = "normal"
    HAPPY = "happy"
    SAD = "sad"
    ANGRY = "angry"
    SURPRISED = "surprised"
    CONFUSED = "confused"
