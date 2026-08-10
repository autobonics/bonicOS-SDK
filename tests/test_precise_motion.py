from __future__ import annotations

import math
import threading
import time
from unittest.mock import patch


def test_drive_distance_reaches_target(robot, transport) -> None:
    transport.set_telemetry(
        "odom", {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )

    def updater() -> None:
        time.sleep(0.05)
        transport.set_telemetry(
            "odom", {"x": 1.0, "y": 0.0, "theta": 0.0, "vx": 0, "vtheta": 0}
        )

    threading.Thread(target=updater, daemon=True).start()
    assert robot.precise.drive_distance(1.0, speed=0.5, timeout=2.0) is True


def test_drive_distance_times_out(robot, transport) -> None:
    transport.set_telemetry(
        "odom", {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )
    assert robot.precise.drive_distance(5.0, speed=0.5, timeout=0.2) is False


def test_drive_distance_without_odom_fails_fast(robot, transport) -> None:
    start = time.monotonic()
    assert robot.precise.drive_distance(1.0, timeout=0.1) is False
    assert (
        time.monotonic() - start < 1.0
    )  # bounded by `timeout`, not ODOM_WAIT_TIMEOUT_S


def test_rotate_angle_accumulates_signed_delta(robot, transport) -> None:
    transport.set_telemetry(
        "odom", {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )

    def updater() -> None:
        time.sleep(0.05)
        transport.set_telemetry(
            "odom",
            {"x": 0.0, "y": 0.0, "theta": math.radians(90), "vx": 0, "vtheta": 0},
        )

    threading.Thread(target=updater, daemon=True).start()
    assert robot.precise.rotate_angle(90.0, speed=90.0, timeout=2.0) is True


def test_rotate_angle_handles_wraparound(robot, transport) -> None:
    # theta jumps from +179 deg to -179 deg — a 2 degree step across the
    # +/-180 boundary, not a near-360 degree spin the naive absolute-delta
    # would compute.
    transport.set_telemetry(
        "odom", {"x": 0.0, "y": 0.0, "theta": math.radians(179), "vx": 0, "vtheta": 0}
    )

    def updater() -> None:
        time.sleep(0.05)
        transport.set_telemetry(
            "odom",
            {"x": 0.0, "y": 0.0, "theta": math.radians(-179), "vx": 0, "vtheta": 0},
        )

    threading.Thread(target=updater, daemon=True).start()
    # Only a 2 degree rotation actually happened — asking for 5 should time out.
    assert robot.precise.rotate_angle(5.0, speed=90.0, timeout=0.3) is False


def test_draw_square_composes_four_legs(robot, transport) -> None:
    with (
        patch.object(robot.precise, "drive_distance", return_value=True) as drive,
        patch.object(robot.precise, "rotate_angle", return_value=True) as rotate,
    ):
        assert robot.precise.draw_square(1.0) is True
        assert drive.call_count == 4
        assert rotate.call_count == 4
        rotate.assert_called_with(90.0, 45.0)


def test_draw_square_stops_on_first_failure(robot, transport) -> None:
    with (
        patch.object(robot.precise, "drive_distance", return_value=False) as drive,
        patch.object(robot.precise, "rotate_angle", return_value=True) as rotate,
    ):
        assert robot.precise.draw_square(1.0) is False
        assert drive.call_count == 1
        assert rotate.call_count == 0


def test_run_queue_blocks_and_reports_success(robot, transport) -> None:
    with (
        patch.object(robot.precise, "drive_distance", return_value=True) as drive,
        patch.object(robot.precise, "rotate_angle", return_value=True) as rotate,
    ):
        robot.precise.enqueue([("drive", 1.0), ("rotate", 90.0)])
        assert robot.precise.run_queue(block=True) is True
        assert drive.call_count == 1
        assert rotate.call_count == 1


def test_clear_queue_empties_pending_commands(robot, transport) -> None:
    with patch.object(robot.precise, "drive_distance", return_value=True) as drive:
        robot.precise.enqueue([("drive", 1.0), ("drive", 1.0), ("drive", 1.0)])
        robot.precise.clear_queue()
        assert robot.precise.run_queue(block=True) is True
        assert drive.call_count == 0
