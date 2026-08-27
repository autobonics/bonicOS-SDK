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


# --- clear_queue must abort the leg already in flight, not just the ones
# --- still queued behind it ------------------------------------------------


def test_clear_queue_aborts_the_leg_already_running(robot, transport) -> None:
    """`clear_queue()` stops the base — and before this was fixed, the leg's
    own loop sent `drive` again a poll later and the robot carried on to
    finish the metre it had been told to abandon. The cancel flag was only
    read *between* queued items.
    """
    transport.set_telemetry(
        "odom", {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )
    # Never reaches the target on its own: only the cancel can end this.
    robot.precise.enqueue([("drive", 100.0), ("drive", 100.0)])

    runner = threading.Thread(
        target=lambda: robot.precise.run_queue(block=True), daemon=True
    )
    runner.start()
    deadline = time.monotonic() + 2.0
    while not any(m["type"] == "drive" for m in transport.sent):
        assert time.monotonic() < deadline, "the leg never started"
        time.sleep(0.01)

    robot.precise.clear_queue()
    runner.join(timeout=2.0)
    assert not runner.is_alive()  # the whole drain unwound, not just this item
    assert transport.sent[-1] == {"type": "drive", "linear_x": 0.0, "angular_z": 0.0}


def test_a_direct_call_after_clear_queue_is_not_cancelled(robot, transport) -> None:
    """The cancel flag stays set after `clear_queue()` returns, so it must be
    scoped to calls the drain itself made."""
    robot.precise.clear_queue()
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
