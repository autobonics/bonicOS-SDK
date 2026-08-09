from __future__ import annotations

import math
import threading
import time

from bonicos import protocol
from bonicos.enums import ServoID

# These first few tests only care about the outgoing payload shape/units, not
# completion — `wait=False` keeps them instant (no ack, no convergence poll)
# rather than blocking on telemetry that's never provided. See below for
# dedicated `wait=True` convergence tests.


def test_set_servos_converts_degrees_to_radians(robot, transport) -> None:
    assert robot.arm.set_servos({"leftElbow": -30.0}, duration=2.0, wait=False) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_SERVO_COMMAND
    assert sent["duration"] == 2.0
    assert math.isclose(sent["servos"]["leftElbow"], math.radians(-30.0))


def test_move_left_arm_uses_shoulder_and_elbow_joints(robot, transport) -> None:
    # The payload is expanded to the FULL left_arm group (ARCHITECTURE.md
    # §4a "_fill_group") — the controller silently ignores a trajectory
    # missing any of its claimed joints. Unspecified siblings default to
    # 0.0 (no telemetry available in this test) via get_servo_angles().
    robot.arm.move_left_arm(shoulder=90, elbow=-30, wait=False)
    servos = transport.sent[-1]["servos"]
    assert set(servos) == set(protocol.JOINT_GROUPS["left_arm"])
    assert math.isclose(servos[ServoID.LEFT_SHOULDER_PITCH.value], math.radians(90))
    assert math.isclose(servos[ServoID.LEFT_ELBOW.value], math.radians(-30))
    assert math.isclose(servos[ServoID.LEFT_SHOULDER_YAW.value], 0.0)


def test_move_right_arm_no_wait_does_not_block_for_ack(robot, transport) -> None:
    # No ack scripted at all — if this waited for one it would raise.
    assert robot.arm.move_right_arm(shoulder=10, elbow=-10, wait=False) is True
    assert transport.sent[-1]["type"] == protocol.CMD_SERVO_COMMAND


def test_open_and_close_grippers(robot, transport) -> None:
    robot.arm.open_grippers()
    servos = transport.sent[-1]["servos"]
    assert servos[ServoID.LEFT_GRIPPER.value] > 0
    robot.arm.close_grippers()
    servos = transport.sent[-1]["servos"]
    assert servos[ServoID.LEFT_GRIPPER.value] < 0


# --- wait=True: real completion (ARCHITECTURE.md §4a) ----------------------
# `wait=True` polls joint_states until the commanded joints converge — it is
# NOT just "wait for the ack" (the ack arrives before the arm even starts
# moving). These mirror test_precise_motion.py's background-thread-pushes-
# telemetry pattern.


def test_move_left_arm_wait_true_blocks_until_convergence(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SERVO_COMMAND, {"ok": True, "unknown": []})

    def updater() -> None:
        time.sleep(0.05)
        # A real /joint_states always reports every joint, not just the ones
        # the caller named — _fill_group holds the rest at 0.0 (no prior
        # telemetry in this test), so all 7 must be present here to converge.
        transport.set_telemetry(
            "joint_states",
            {
                "name": [
                    "left_shoulder_yaw_joint",
                    "left_shoulder_roll_joint",
                    "left_shoulder_pitch_joint",
                    "left_elbow_joint",
                    "left_wrist_yaw_joint",
                    "left_wrist_pitch_joint",
                    "left_gripper_yaw_joint",
                ],
                "position": [
                    0.0,
                    0.0,
                    math.radians(90),
                    math.radians(-30),
                    0.0,
                    0.0,
                    0.0,
                ],
            },
        )

    threading.Thread(target=updater, daemon=True).start()
    assert robot.arm.move_left_arm(shoulder=90, elbow=-30, timeout=2.0) is True


def test_move_left_arm_wait_true_times_out_if_never_converges(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SERVO_COMMAND, {"ok": True, "unknown": []})
    # No matching telemetry ever arrives — must time out, not hang.
    start = time.monotonic()
    assert robot.arm.move_left_arm(shoulder=90, elbow=-30, timeout=0.2) is False
    assert time.monotonic() - start < 1.0


def test_send_servo_command_returns_false_fast_when_ack_not_ok(
    robot, transport
) -> None:
    transport.script_ack(protocol.CMD_SERVO_COMMAND, {"ok": False})
    start = time.monotonic()
    assert robot.arm.set_servos({"leftElbow": -30.0}, timeout=5.0) is False
    # Must short-circuit on the failed ack, never enter the convergence poll.
    assert time.monotonic() - start < 1.0


def test_send_servo_command_excludes_unknown_keys_from_convergence(
    robot, transport
) -> None:
    # "bogus" is reported unknown by the server — it was never actually sent
    # to a joint, so it must not be waited on (it would never converge and
    # would spuriously time out the whole call otherwise).
    transport.script_ack(protocol.CMD_SERVO_COMMAND, {"ok": True, "unknown": ["bogus"]})

    def updater() -> None:
        time.sleep(0.05)
        # "leftElbow" pulls in the rest of the left_arm group (_fill_group);
        # a real /joint_states reports all of them, so the fake one must too.
        transport.set_telemetry(
            "joint_states",
            {
                "name": [
                    "left_shoulder_yaw_joint",
                    "left_shoulder_roll_joint",
                    "left_shoulder_pitch_joint",
                    "left_elbow_joint",
                    "left_wrist_yaw_joint",
                    "left_wrist_pitch_joint",
                    "left_gripper_yaw_joint",
                ],
                "position": [0.0, 0.0, 0.0, math.radians(-30), 0.0, 0.0, 0.0],
            },
        )

    threading.Thread(target=updater, daemon=True).start()
    assert robot.arm.set_servos({"leftElbow": -30.0, "bogus": 1.0}, timeout=2.0) is True


def test_send_servo_command_all_unknown_keys_returns_true_without_waiting(
    robot, transport
) -> None:
    transport.script_ack(protocol.CMD_SERVO_COMMAND, {"ok": True, "unknown": ["bogus"]})
    start = time.monotonic()
    assert robot.arm.set_servos({"bogus": 1.0}, timeout=5.0) is True
    assert time.monotonic() - start < 1.0


# --- _fill_group: full-group-vector requirement (ARCHITECTURE.md §4a) ------
# left_arm/right_arm/head controllers silently ignore a command missing any
# of their claimed joints (verified against the real M1 sim + an independent
# ROS-level stress test, 2026-08-04) — a partial servo_command is a no-op on
# real hardware even though it acks `ok: true`.


def test_fill_group_holds_unspecified_joints_at_current_position(
    robot, transport
) -> None:
    # Prior telemetry shows the arm already displaced from zero.
    transport.set_telemetry(
        "joint_states",
        {"name": ["left_shoulder_yaw_joint"], "position": [math.radians(15.0)]},
    )
    robot.arm.move_left_arm(shoulder=90, elbow=-30, wait=False)
    servos = transport.sent[-1]["servos"]
    assert set(servos) == set(protocol.JOINT_GROUPS["left_arm"])
    # Not overwritten by the fill — held at its last known real position.
    assert math.isclose(servos[ServoID.LEFT_SHOULDER_YAW.value], math.radians(15.0))
    # Joints truly never seen fall back to 0.0.
    assert math.isclose(servos[ServoID.LEFT_WRIST_YAW.value], 0.0)


def test_fill_group_leaves_single_joint_groups_untouched(robot, transport) -> None:
    # left_gripper/right_gripper each own exactly one joint — nothing to fill.
    robot.arm.open_grippers()
    servos = transport.sent[-1]["servos"]
    assert set(servos) == {ServoID.LEFT_GRIPPER.value, ServoID.RIGHT_GRIPPER.value}


def test_fill_group_ignores_keys_outside_any_known_group(robot, transport) -> None:
    robot.arm.set_servos({"totallyMadeUp": 1.0}, wait=False)
    servos = transport.sent[-1]["servos"]
    assert set(servos) == {"totallyMadeUp"}


def test_set_single_servo_stub(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SERVO_SINGLE, {"ok": True})
    assert robot.arm.set_single_servo("headTilt", 10.0) is True
    sent = transport.sent[-1]
    assert "id" in sent
    assert sent["type"] == protocol.CMD_SERVO_SINGLE
    assert sent["joint"] == "headTilt"
    assert sent["angle"] == 10.0


def test_get_servo_angles_reads_joint_states_in_degrees(robot, transport) -> None:
    # joint_states reports raw snake_case URDF names on the wire (PROTOCOL.md
    # §6), never the camelCase registry keys a command is sent with —
    # get_servo_angles() must translate back via protocol.JOINT_NAME_MAP.
    transport.set_telemetry(
        "joint_states",
        {
            "name": ["left_elbow_joint", "neck_yaw_joint", "left_wheel_joint"],
            "position": [math.radians(45), math.radians(-10), math.radians(999)],
        },
    )
    angles = robot.arm.get_servo_angles()
    assert math.isclose(angles["leftElbow"], 45.0)
    assert math.isclose(angles["neckYaw"], -10.0)
    # left_wheel_joint isn't in the servo registry — must not leak through.
    assert "left_wheel_joint" not in angles
    assert len(angles) == 2


def test_get_servo_angles_empty_without_telemetry(robot, transport) -> None:
    assert robot.arm.get_servo_angles() == {}
