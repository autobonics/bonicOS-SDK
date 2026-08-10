"""SimTransport (Implementation brief Task 5 — dev/ARCHITECTURE.md §7).

Unlike ``MockTransport``, telemetry here isn't scripted by the test — it's
computed by real (if simplified) physics, so these tests drive the
transport the way a user's code would and check the numbers that come back.
"""

from __future__ import annotations

import math
import time

import pytest

from bonicos import protocol
from bonicos.exceptions import CameraUnavailable
from bonicos.transports.sim import SimTransport
from tests.conftest import FakeRobot


@pytest.fixture
def sim() -> SimTransport:
    return SimTransport()


def test_drive_forward_for_known_duration_matches_geometry(sim: SimTransport) -> None:
    speed = 0.5
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": speed, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 0.3:
        sim.wait_for_update(1.0)
    elapsed = time.monotonic() - start
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})

    odom = sim.read_telemetry()["odom"]
    # Expected distance from *measured* elapsed time, not a nominal
    # duration — the sim integrates wall-clock time, so this holds
    # regardless of how fast the busy loop above actually spun.
    assert odom["x"] == pytest.approx(speed * elapsed, abs=0.03)
    assert odom["y"] == pytest.approx(0.0, abs=1e-9)


def test_drive_rotation_updates_heading(sim: SimTransport) -> None:
    angular_speed = math.radians(90)
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": angular_speed})
    start = time.monotonic()
    while time.monotonic() - start < 0.2:
        sim.wait_for_update(1.0)
    elapsed = time.monotonic() - start
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})

    assert sim.read_telemetry()["pose"]["theta"] == pytest.approx(
        angular_speed * elapsed, abs=0.05
    )


def test_wait_for_update_returns_true_and_advances_time_single_threaded(
    sim: SimTransport,
) -> None:
    # No second thread anywhere in this test — SimTransport must not block
    # on a threading.Event the way MockTransport does (dev/ARCHITECTURE.md
    # §3.2): one call, one tick, always True, even with nothing else running.
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 1.0, "angular_z": 0.0})
    before = sim.read_telemetry()["odom"]["x"]
    time.sleep(0.05)
    assert sim.wait_for_update(5.0) is True
    after = sim.read_telemetry()["odom"]["x"]
    assert after > before  # the sleep's wall-clock time was integrated, not lost
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})


def test_servo_command_converges_and_reads_back_in_camelcase(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    assert (
        robot.arm.move_left_arm(shoulder=45, elbow=-30, duration=0.05, timeout=1.0)
        is True
    )
    # `move_left_arm` returns as soon as it's within its own convergence
    # tolerance (8.6°), which can land before the ramp's nominal duration
    # has fully elapsed — let it settle the rest of the way before checking
    # the exact value, same as `get_servo_angles()` would read moments later.
    time.sleep(0.1)
    sim.wait_for_update(0.0)
    angles = robot.arm.get_servo_angles()  # camelCase — same keys the call used
    assert angles["leftShoulderPitch"] == pytest.approx(45.0, abs=0.5)
    assert angles["leftElbow"] == pytest.approx(-30.0, abs=0.5)


def test_servo_ramp_is_preempted_by_a_new_target(sim: SimTransport) -> None:
    # Mirrors the verified real-hardware behaviour (dev/ARCHITECTURE.md
    # §4a): a command to a joint already mid-ramp restarts smoothly from
    # its *current interpolated position*, not from 0 or the old target.
    sim.send(
        {
            "type": protocol.CMD_SERVO_COMMAND,
            "servos": {"neckYaw": math.radians(90)},
            "duration": 1.0,
        }
    )
    time.sleep(0.1)
    mid = sim.read_telemetry()["joint_states"]
    mid_yaw = dict(zip(mid["name"], mid["position"]))["neck_yaw_joint"]
    assert 0.0 < mid_yaw < math.radians(90)  # partway there, not 0 and not done

    sim.send(
        {
            "type": protocol.CMD_SERVO_COMMAND,
            "servos": {"neckYaw": math.radians(-45)},
            "duration": 0.05,
        }
    )
    time.sleep(0.15)  # comfortably past the new, short duration
    final = sim.read_telemetry()["joint_states"]
    final_yaw = dict(zip(final["name"], final["position"]))["neck_yaw_joint"]
    assert final_yaw == pytest.approx(math.radians(-45), abs=0.05)


def test_unknown_joint_key_is_reported_and_not_applied(sim: SimTransport) -> None:
    cmd_id = sim.send(
        {
            "type": protocol.CMD_SERVO_COMMAND,
            "servos": {"totallyMadeUp": 1.0},
            "duration": 0.1,
        }
    )
    ack = sim.wait_for_ack(cmd_id)
    assert ack["ok"] is True
    assert ack["unknown"] == ["totallyMadeUp"]


def test_supports_camera_false_and_camera_api_raises(sim: SimTransport) -> None:
    assert sim.supports_camera is False
    with pytest.raises(CameraUnavailable):
        sim.start_camera(["main"])
    # Must raise, not just return None forever with no signal that there's
    # no camera path at all (Implementation brief Task 5).
    assert sim.read_frame() is None


def test_navigation_and_mapping_ack_and_do_nothing(sim: SimTransport) -> None:
    # No Nav2/SLAM behind this transport — matches the real robot's own
    # stub convention (Implementation brief Task 5).
    goal_id = sim.send(
        {"type": protocol.CMD_NAV_GOAL, "x": 5.0, "y": 5.0, "theta": 0.0}
    )
    sim.wait_for_ack(goal_id)  # acked, doesn't raise
    assert sim.read_telemetry().get(protocol.EVENT_NAV_STATUS) is None  # never faked

    save_id = sim.send({"type": protocol.CMD_SAVE_MAP, "name": "kitchen"})
    assert sim.wait_for_ack(save_id)["ok"] is True

    list_id = sim.send({"type": protocol.CMD_LIST_MAPS})
    assert sim.wait_for_ack(list_id)["maps"] == [
        {"name": "kitchen", "size": 0, "modified": 0}
    ]


def test_get_state_reports_pose_and_joints(sim: SimTransport) -> None:
    state = sim.get_state()
    assert set(state) == {"pose", "joints"}
    assert set(state["pose"]) == {"x", "y", "theta"}
    assert "left_elbow_joint" in state["joints"]  # snake_case URDF names
