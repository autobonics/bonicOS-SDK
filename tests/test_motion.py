from __future__ import annotations

import time

from bonicos import protocol


def test_drive_sends_no_id(robot, transport) -> None:
    robot.motion.drive(0.5, -0.2)
    assert transport.sent[-1] == {
        "type": protocol.CMD_DRIVE,
        "linear_x": 0.5,
        "angular_z": -0.2,
    }
    robot.motion.stop()  # stop the keepalive thread so it doesn't outlive the test


def test_move_forward_uses_positive_speed(robot, transport) -> None:
    robot.motion.move_forward(0.4)
    assert transport.sent[-1]["linear_x"] == 0.4
    robot.motion.stop()


def test_move_backward_negates_speed(robot, transport) -> None:
    robot.motion.move_backward(0.4)
    assert transport.sent[-1]["linear_x"] == -0.4
    robot.motion.stop()


def test_turn_left_is_positive_angular_z(robot, transport) -> None:
    robot.motion.turn_left(0.7)
    assert transport.sent[-1]["angular_z"] == 0.7
    robot.motion.stop()


def test_turn_right_is_negative_angular_z(robot, transport) -> None:
    robot.motion.turn_right(0.7)
    assert transport.sent[-1]["angular_z"] == -0.7
    robot.motion.stop()


def test_duration_blocks_then_stops(robot, transport) -> None:
    start = time.monotonic()
    robot.motion.move_forward(0.3, duration=0.1)
    elapsed = time.monotonic() - start
    assert elapsed >= 0.1
    assert transport.sent[-1] == {
        "type": protocol.CMD_DRIVE,
        "linear_x": 0.0,
        "angular_z": 0.0,
    }


def test_duration_none_fires_and_returns(robot, transport) -> None:
    start = time.monotonic()
    robot.motion.move_forward(0.3, duration=None)
    elapsed = time.monotonic() - start
    assert elapsed < 0.1
    robot.motion.stop()


def test_is_moving_reads_odom(robot, transport) -> None:
    assert robot.motion.is_moving() is False
    transport.set_telemetry("odom", {"x": 0, "y": 0, "theta": 0, "vx": 0.5, "vtheta": 0.0})
    assert robot.motion.is_moving() is True


def test_zero_drive_does_not_keep_alive(robot, transport) -> None:
    robot.motion.drive(0.0, 0.0)
    count_before = len(transport.sent)
    time.sleep(0.3)
    assert len(transport.sent) == count_before  # no keepalive resends for a stopped robot


def test_nonzero_drive_keeps_alive_under_the_deadman(robot, transport) -> None:
    robot.motion.drive(0.3, 0.0)
    time.sleep(0.4)  # > 2x the 150ms keepalive interval
    robot.motion.stop()
    drive_frames = [m for m in transport.sent if m["type"] == protocol.CMD_DRIVE]
    assert len(drive_frames) >= 3  # initial + at least two keepalive resends
