"""SimTransport — the physics-backed fake robot.

Unlike ``MockTransport``, telemetry here isn't scripted by the test — it's
computed by real (if simplified) physics, so these tests drive the
transport the way a user's code would and check the numbers that come back.
"""

from __future__ import annotations

import math
import time

import pytest

import bonicos
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
    # on a threading.Event the way MockTransport does: one call, one tick,
    # always True, even with nothing else running.
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
    # Mirrors the verified real-hardware behaviour: a command to a joint
    # already mid-ramp restarts smoothly from its *current interpolated
    # position*, not from 0 or the old target.
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
    # no camera path at all.
    assert sim.read_frame() is None


def test_mapping_acks_and_does_nothing(sim: SimTransport) -> None:
    # No SLAM behind this transport — matches the real robot's own stub
    # convention. (Navigation is NOT a stub — see the nav_goal/A*/Regulated
    # Pure Pursuit tests below.)
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


# --- dev/SIMULATOR.md §6: obstacle model -------------------------------------------------


def test_obstacles_normalize_to_half_extent_footprints() -> None:
    sim = SimTransport(
        obstacles=[
            {"id": "b1", "x": 1.0, "y": 2.0, "sizeX": 0.4, "sizeY": 0.6, "height": 0.3}
        ]
    )
    (obs,) = sim._obstacles
    assert (obs.cx, obs.cy, obs.hx, obs.hy) == (1.0, 2.0, 0.2, 0.3)
    # `theta` is optional on the wire: a host that never sends one gets the
    # axis-aligned box it always got.
    assert obs.theta == 0.0
    assert obs.bounds() == (0.8, 1.7, 1.2, 2.3)


def test_bonicbot_simulated_forwards_obstacles_to_the_transport() -> None:
    # §4.4: `.simulated()` is a legitimate simulator-only *constructor* (not
    # a simulator-only robot API), so it must forward `obstacles` straight
    # through, same as it already forwards `joints`.
    robot = bonicos.BonicBot.simulated(
        obstacles=[
            {"id": "b1", "x": 1.0, "y": 2.0, "sizeX": 0.4, "sizeY": 0.6, "height": 0.3}
        ]
    )
    assert robot._transport._obstacles[0].bounds() == (0.8, 1.7, 1.2, 2.3)
    bonicos.use_transport(None)  # don't leak the sticky registration to other tests


def test_omitting_obstacles_yields_empty_list_and_unchanged_behaviour(
    sim: SimTransport,
) -> None:
    assert sim._obstacles == []
    # No obstacle in the way: driving forward behaves exactly as before
    # collision existed.
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.3, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 0.2:
        sim.wait_for_update(1.0)
    elapsed = time.monotonic() - start
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    assert sim.read_telemetry()["odom"]["x"] == pytest.approx(0.3 * elapsed, abs=0.03)


# --- dev/SIMULATOR.md §3.1: base collision --------------------------------------------------


@pytest.fixture
def sim_with_box() -> SimTransport:
    # A 0.4x0.4 box centred on (1.0, 0.0) — face at x=0.8.
    return SimTransport(
        obstacles=[
            {"id": "b1", "x": 1.0, "y": 0.0, "sizeX": 0.4, "sizeY": 0.4, "height": 0.3}
        ]
    )


def test_drive_forward_into_box_stops_at_its_face_and_does_not_pass_through(
    sim_with_box: SimTransport,
) -> None:
    sim = sim_with_box
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 3.0:
        sim.wait_for_update(0.1)
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    x = sim.read_telemetry()["pose"]["x"]
    # Box face at 0.8, robot radius 0.22 -> should stop around 0.58, and
    # never reach the box's own x (1.0), let alone pass through it.
    assert x == pytest.approx(0.8 - 0.22, abs=0.02)
    assert x < 0.8


def test_drive_rotation_still_works_while_pinned_against_a_box(
    sim_with_box: SimTransport,
) -> None:
    sim = sim_with_box
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 3.0:
        sim.wait_for_update(0.1)
    # Now pinned against the box's face. Ask for translation + rotation —
    # translation should be refused but rotation should still happen.
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 1.0})
    x_before = sim.read_telemetry()["pose"]["x"]
    start = time.monotonic()
    while time.monotonic() - start < 0.5:
        sim.wait_for_update(0.1)
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    telem = sim.read_telemetry()
    assert telem["pose"]["theta"] != pytest.approx(0.0, abs=1e-3)
    assert telem["pose"]["x"] == pytest.approx(x_before, abs=0.05)  # still pinned


# --- dev/SIMULATOR.md §3.1/§3.5: rotated (oriented) obstacles ----------------------------


def test_rotated_obstacle_reaches_further_in_world_axes_than_its_own_extents() -> None:
    # A 1.0 x 0.2 wall laid on the diagonal. In its own frame it is still
    # 1.0 x 0.2; in world x/y it now spans 0.849 both ways. Grid sizing
    # reads `bounds()` for exactly this reason.
    sim = SimTransport(
        obstacles=[
            {
                "id": "w",
                "x": 0.0,
                "y": 0.0,
                "sizeX": 1.0,
                "sizeY": 0.2,
                "theta": math.radians(45),
            }
        ]
    )
    (obs,) = sim._obstacles
    assert (obs.hx, obs.hy) == (0.5, 0.1)
    reach = 0.5 * math.cos(math.radians(45)) + 0.1 * math.sin(math.radians(45))
    assert obs.bounds() == pytest.approx((-reach, -reach, reach, reach))


def test_collision_uses_the_oriented_box_not_its_enclosing_aabb() -> None:
    # The discriminating case: a 1.0 x 0.2 wall at 45 deg centred at
    # (1.5, 0). Its enclosing AABB starts at x=1.076, but most of that box
    # is empty floor the robot is entitled to drive into.
    sim = SimTransport(
        obstacles=[
            {
                "id": "w",
                "x": 1.5,
                "y": 0.0,
                "sizeX": 1.0,
                "sizeY": 0.2,
                "height": 0.5,
                "theta": math.radians(45),
            }
        ]
    )
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 4.0:
        sim.wait_for_update(0.1)
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})

    x = sim.read_telemetry()["pose"]["x"]
    # Circle-vs-OBB puts the stop at 1.048 (solved against the tilted face).
    assert x == pytest.approx(1.048, abs=0.02)
    # Had it fallen back to the enclosing AABB the robot would have stopped
    # at 0.856 — nearly 0.2 m short, in front of nothing at all.
    assert x > 0.9
    # And it is still outside the wall: along y=0 the surface is at 1.359.
    assert x < 1.359


def test_sub_cell_width_diagonal_wall_rasterizes_without_gaps() -> None:
    # A wall thinner than one grid cell (0.04 < 0.05), on the diagonal.
    # Testing bare cell centres would mark only the cells whose middle the
    # wall happens to cross, leaving a dotted line with holes between them.
    # 30 deg, deliberately not 45: on the exact diagonal the wall's
    # centreline runs straight through cell centres, so even a bare
    # centre test marks every cell and the bug hides completely. Any
    # other angle exposes it — this one leaves 30 of 77 cells unmarked.
    length, theta = 3.0, math.radians(30)
    sim = SimTransport(
        obstacles=[
            {
                "id": "w",
                "x": 0.0,
                "y": 0.0,
                "sizeX": length,
                "sizeY": 0.04,
                "theta": theta,
            }
        ]
    )
    # Walk the wall's centreline in steps well under a cell and require
    # every sample to land in an occupied cell of the RAW grid.
    steps = int(length / (0.05 * 0.25))
    for i in range(steps + 1):
        t = -length / 2 + length * i / steps
        row, col = sim._world_to_cell(t * math.cos(theta), t * math.sin(theta))
        assert sim._occupied(sim._grid_raw, row, col), f"gap at t={t:.3f}"


def test_navigation_routes_around_a_rotated_wall_without_clipping_it() -> None:
    # End to end: the planner reads the same rotated footprint the collision
    # check does, so a goal behind a diagonal wall is reached around it.
    theta = math.radians(45)
    sim = SimTransport(
        obstacles=[
            {
                "id": "w",
                "x": 1.2,
                "y": 0.0,
                "sizeX": 2.0,
                "sizeY": 0.15,
                "height": 0.5,
                "theta": theta,
            }
        ]
    )
    sim.send({"type": protocol.CMD_NAV_GOAL, "x": 2.4, "y": 0.0, "theta": 0.0})
    planned: list = []
    start = time.monotonic()
    while time.monotonic() - start < 45.0:
        sim.wait_for_update(0.1)
        telem = sim.read_telemetry()
        # The plan is cleared to [] on arrival, so grab it while it is live
        # rather than reading an empty one back at the end.
        planned = planned or telem.get("plan", {}).get("points", [])
        if telem.get("nav_status", {}).get("status") in (
            "succeeded",
            "failed",
            "canceled",
        ):
            break
    telem = sim.read_telemetry()
    assert telem["nav_status"]["status"] == "succeeded"
    assert (telem["pose"]["x"], telem["pose"]["y"]) == pytest.approx(
        (2.4, 0.0), abs=0.15
    )
    # The path must stay out of the wall itself, not merely end up on the
    # far side of it.
    (obs,) = sim._obstacles
    assert planned
    for px, py in planned:
        assert not obs.contains(px, py), f"plan clips the wall at ({px}, {py})"


# --- dev/SIMULATOR.md §6: per-series hardware (radius, cameras) --------------------------


def test_radius_defaults_to_the_m_series_value() -> None:
    # The default is the calibrated M1 number, so every existing caller and
    # every world built before the three series were modelled is unaffected.
    sim = SimTransport()
    assert sim._radius == 0.22
    assert sim._inflation == pytest.approx(0.27)


def test_a_smaller_robot_drives_closer_to_a_box_before_stopping() -> None:
    # Collision is circle-vs-footprint, so the stopping distance IS the
    # radius. An 'a' series robot should get visibly nearer than an 'm'.
    stops = {}
    for radius in (0.16, 0.22):
        sim = SimTransport(
            radius=radius,
            obstacles=[{"id": "b1", "x": 1.0, "y": 0.0, "sizeX": 0.4, "sizeY": 0.4}],
        )
        sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            sim.wait_for_update(0.1)
        sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
        stops[radius] = sim.read_telemetry()["pose"]["x"]
    # Box face at 0.8; each stops one radius short of it.
    assert stops[0.16] == pytest.approx(0.8 - 0.16, abs=0.02)
    assert stops[0.22] == pytest.approx(0.8 - 0.22, abs=0.02)
    assert stops[0.16] > stops[0.22]


def test_radius_also_widens_the_planner_inflation() -> None:
    # Not just collision: the costmap has to grow too, or a small robot is
    # refused gaps it fits through and a large one is routed into walls.
    small = SimTransport(
        radius=0.16,
        obstacles=[{"id": "b", "x": 1.0, "y": 0.0, "sizeX": 0.4, "sizeY": 0.4}],
    )
    large = SimTransport(
        radius=0.30,
        obstacles=[{"id": "b", "x": 1.0, "y": 0.0, "sizeX": 0.4, "sizeY": 0.4}],
    )
    assert large._inflation > small._inflation
    assert sum(1 for v in large._grid_inflated if v) > sum(
        1 for v in small._grid_inflated if v
    )


def test_cameras_default_to_a_single_main_camera() -> None:
    sim = SimTransport()
    sim.set_frame_provider(lambda *a: None)
    assert sim._auth_result["cameras"] == ["main"]


def test_camera_names_come_from_the_host_and_reach_list_cameras() -> None:
    # The names must be the ROBOT's names — an M1 reports face + docking
    # (bonicOS-robot-app/app/config.py), so the simulator must too, or
    # `get_frame("face")` works in one place and not the other.
    sim = SimTransport(cameras=["face", "docking"])
    sim.set_frame_provider(lambda *a: None)
    assert sim._auth_result["cameras"] == ["face", "docking"]

    bonicos.use_transport(sim)
    robot = bonicos.BonicBot()
    assert robot.camera.list() == ["face", "docking"]
    bonicos.use_transport(None)


def test_each_started_camera_renders_and_reads_back_independently() -> None:
    rendered: list = []

    def provider(camera, x, y, theta, neck_yaw, neck_pitch):
        rendered.append(camera)
        # One distinct pixel per camera, so a crossed wire is visible.
        value = 1 if camera == "face" else 2
        return (bytes([value, value, value]), 1, 1)

    sim = SimTransport(cameras=["face", "docking"])
    sim.set_frame_provider(provider)
    sim.start_camera(["face", "docking"])
    sim.wait_for_update(1.0)

    assert sorted(set(rendered)) == ["docking", "face"]
    assert sim.read_frame("face")[0][0][0] == 1
    assert sim.read_frame("docking")[0][0][0] == 2
    # No argument takes the first camera, same as on a real robot.
    assert sim.read_frame()[0][0][0] == 1


def test_an_unstarted_camera_is_not_readable_even_while_another_runs() -> None:
    sim = SimTransport(cameras=["face", "docking"])
    sim.set_frame_provider(lambda *a: (bytes([7, 7, 7]), 1, 1))
    sim.start_camera(["face"])
    sim.wait_for_update(1.0)

    assert sim.read_frame("face") is not None
    # Never started: a real robot has no track for it, so neither has this.
    assert sim.read_frame("docking") is None


def test_preview_alone_renders_only_the_first_camera_and_reads_back_none() -> None:
    # Gating table, dev/SIMULATOR.md §3.6: preview open renders (so the panel
    # is live) but `read_frame` stays None without `start_camera()`. With two
    # cameras it must not render both — the panel only shows one.
    rendered: list = []

    def provider(camera, *rest):
        rendered.append(camera)
        return (bytes([3, 3, 3]), 1, 1)

    sim = SimTransport(cameras=["face", "docking"])
    sim.set_frame_provider(provider)
    sim.set_camera_preview_open(True)
    sim.wait_for_update(1.0)

    assert rendered == ["face"]
    assert sim.read_frame("face") is None


# --- dev/SIMULATOR.md §3.3: synthesized IMU -------------------------------------------------


def test_stationary_imu_reports_only_gravity(sim: SimTransport) -> None:
    telem = sim.read_telemetry()
    imu = telem["imu"]
    assert imu["az"] == pytest.approx(9.81)
    for key in ("ax", "ay", "gx", "gy", "gz"):
        assert imu[key] == pytest.approx(0.0)


def test_drive_forward_gives_ax_spike_then_settles(sim: SimTransport) -> None:
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.3, "angular_z": 0.0})
    time.sleep(0.05)
    ax_spike = sim.read_telemetry()["imu"]["ax"]  # single tick: real elapsed dt
    assert ax_spike > 0.5  # accelerating from rest

    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        time.sleep(0.05)
        telem = sim.read_telemetry()
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    assert telem["imu"]["ax"] == pytest.approx(0.0, abs=0.2)  # steady speed now


def test_curved_drive_reports_yaw_rate_and_centripetal_ay(sim: SimTransport) -> None:
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.3, "angular_z": 0.5})
    telem = sim.read_telemetry()
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    assert telem["imu"]["gz"] == pytest.approx(0.5, abs=1e-6)
    assert telem["imu"]["ay"] == pytest.approx(0.15, abs=1e-6)


def test_driving_into_a_wall_gives_negative_ax_spike(
    sim_with_box: SimTransport,
) -> None:
    sim = sim_with_box
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
    prev_vx = None
    saw_negative_spike = False
    start = time.monotonic()
    while time.monotonic() - start < 3.0:
        time.sleep(0.03)
        telem = sim.read_telemetry()
        vx = telem["odom"]["vx"]
        if prev_vx is not None and prev_vx > 0.0 and vx == 0.0:
            saw_negative_spike = telem["imu"]["ax"] < 0.0
            break
        prev_vx = vx
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    assert saw_negative_spike


# --- dev/SIMULATOR.md §3.4: navigation (A* -> smoothing -> Regulated Pure Pursuit) --------


def test_go_to_on_empty_floor_drives_straight_and_arrives(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    start = time.monotonic()
    assert robot.nav.go_to(2.0, 0.0, wait=True, timeout=20.0) is True
    elapsed = time.monotonic() - start
    # NAV_V_MAX = 0.35 m/s -> ~5.7s nominal, generous slack either side.
    assert 2.0 < elapsed < 15.0
    pos = robot.sensors.get_position()
    assert pos["x"] == pytest.approx(2.0, abs=0.15)
    assert pos["y"] == pytest.approx(0.0, abs=0.1)


def test_go_to_reports_moving_then_succeeded(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    robot.nav.go_to(1.5, 0.0, wait=False)
    saw_moving = False
    start = time.monotonic()
    while time.monotonic() - start < 20.0:
        sim.wait_for_update(0.2)
        status = robot.nav.get_nav_status()
        if status == "moving":
            saw_moving = True
        if status in ("succeeded", "failed", "canceled"):
            break
    assert saw_moving
    assert robot.nav.get_nav_status() == "succeeded"


def test_go_to_arcs_around_a_box_without_ever_stopping_to_turn(
    sim_with_box: SimTransport,
) -> None:
    sim = sim_with_box
    robot = FakeRobot(sim)
    robot.nav.go_to(2.0, 0.0, wait=False)

    departed = False
    stuttered = False
    start = time.monotonic()
    while time.monotonic() - start < 20.0:
        sim.wait_for_update(0.1)
        # Both fields must come from the SAME telemetry snapshot (one tick)
        # — reading `nav_status` and `odom` via two separate calls straddles
        # two different ticks and can pair a stale "moving" against a
        # fresher, already-arrived vx==0, which looks like a stutter but
        # isn't one.
        telem = sim.read_telemetry()
        status = telem.get("nav_status", {}).get("status")
        vx = telem["odom"]["vx"]
        if not departed and vx != 0.0:
            departed = True
        elif departed and status == "moving" and vx == 0.0:
            stuttered = True  # regression: mid-path stop-and-turn
        if status in ("succeeded", "failed", "canceled"):
            break

    assert status == "succeeded"
    assert not stuttered
    pos = robot.sensors.get_position()
    assert pos["x"] == pytest.approx(2.0, abs=0.15)
    # Went around the box, not through it: obstacle spans x in [0.8, 1.2] at
    # y=0, so the path must have swung away from y=0 at some point — checked
    # indirectly by the "never touched the box" collision guarantee (dev/SIMULATOR.md §3.1
    # tests) plus arrival at the far side.


def test_go_to_goal_inside_obstacle_fails_fast(sim_with_box: SimTransport) -> None:
    robot = FakeRobot(sim_with_box)
    start = time.monotonic()
    assert robot.nav.go_to(1.0, 0.0, wait=True, timeout=60.0) is False
    elapsed = time.monotonic() - start
    assert elapsed < 1.0  # not the 60s timeout


def test_get_nav_status_idle_before_any_goal(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    assert robot.nav.get_nav_status() == "idle"


def test_cancel_goal_mid_travel_stops_the_robot(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    robot.nav.go_to(3.0, 0.0, wait=False)
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        sim.wait_for_update(0.1)
    assert robot.nav.cancel_goal() is True
    assert robot.nav.wait_for_goal(timeout=1.0) is False
    assert sim.read_telemetry()["odom"]["vx"] == 0.0


def test_manual_drive_mid_goal_cancels_navigation_and_takes_control(
    sim: SimTransport,
) -> None:
    robot = FakeRobot(sim)
    robot.nav.go_to(3.0, 0.0, wait=False)
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        sim.wait_for_update(0.1)
    robot.motion.drive(0.1, 0.0)
    assert robot.nav.get_nav_status() == "canceled"


# --- dev/SIMULATOR.md §5: plan/map/costmap telemetry --------------------------------------


def test_get_plan_empty_when_idle(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    assert robot.nav.get_plan() == []


def test_get_plan_nonempty_during_travel_and_cleared_on_success(
    sim: SimTransport,
) -> None:
    robot = FakeRobot(sim)
    robot.nav.go_to(2.0, 0.0, wait=False)
    sim.wait_for_update(0.1)
    plan = robot.nav.get_plan()
    assert len(plan) > 0
    # The published plan is the smoothed path: it starts at the origin and
    # ends at the goal, same as `_plan_path`'s contract.
    assert plan[0] == pytest.approx((0.0, 0.0), abs=1e-6)
    assert plan[-1] == pytest.approx((2.0, 0.0), abs=1e-6)

    assert robot.nav.wait_for_goal(timeout=20.0) is True
    assert robot.nav.get_plan() == []


def test_get_plan_cleared_on_cancel(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    robot.nav.go_to(3.0, 0.0, wait=False)
    sim.wait_for_update(0.1)
    assert len(robot.nav.get_plan()) > 0
    robot.nav.cancel_goal()
    assert robot.nav.get_plan() == []


def test_get_plan_cleared_on_planning_failure(sim_with_box: SimTransport) -> None:
    robot = FakeRobot(sim_with_box)
    assert robot.nav.go_to(1.0, 0.0, wait=True, timeout=5.0) is False
    assert robot.nav.get_plan() == []


def test_get_map_and_costmap_same_dimensions_costmap_more_occupied(
    sim_with_box: SimTransport,
) -> None:
    robot = FakeRobot(sim_with_box)
    grid_map = robot.nav.get_map()
    costmap = robot.nav.get_costmap()
    assert grid_map is not None and costmap is not None
    assert grid_map["info"]["width"] == costmap["info"]["width"]
    assert grid_map["info"]["height"] == costmap["info"]["height"]
    assert (
        len(grid_map["data"]) == grid_map["info"]["width"] * grid_map["info"]["height"]
    )
    assert len(costmap["data"]) == len(grid_map["data"])
    # Inflation grows the box's footprint, so strictly more cells register
    # occupied on the costmap than on the raw map.
    assert costmap["data"].count(100) > grid_map["data"].count(100)


def test_get_map_empty_floor_has_no_occupied_cells(sim: SimTransport) -> None:
    robot = FakeRobot(sim)
    grid_map = robot.nav.get_map()
    assert grid_map is not None
    assert grid_map["data"].count(100) == 0


# --- Phase 2 / dev/SIMULATOR.md §3.6: camera frame provider seam ---------------------------
#
# No browser, no OffscreenCanvas here — these drive `SimTransport`'s half of
# the seam with a fake provider, exactly the way `simWorker.ts` drives the
# real one (dev/SIMULATOR.md §3.6). The provider's own rendering is
# dev/SIMULATOR.md §3.6, exercised only in the browser.


def _make_provider():
    """A fake frame provider recording every call, returning a small,
    recognisable BGR image so pixel order survives the round trip through
    `read_frame`."""
    calls = []

    def provider(
        camera: str,
        x: float,
        y: float,
        theta: float,
        neck_yaw: float,
        neck_pitch: float,
    ):
        calls.append(camera)
        width, height = 2, 2
        # BGR bytes, row-major: a distinct value per channel per pixel so a
        # transposition or channel swap would be caught by the assertions.
        bgr = bytes([10, 20, 30] * (width * height))
        return bgr, width, height

    provider.calls = calls
    return provider


def test_read_frame_none_before_start_camera(sim: SimTransport) -> None:
    sim.set_frame_provider(_make_provider())
    assert sim.read_frame() is None  # start_camera() not called yet


def test_start_camera_raises_without_provider(sim: SimTransport) -> None:
    # No set_frame_provider call at all — same contract as the pre-Phase-2
    # stub: no camera path, no silent None-forever.
    with pytest.raises(CameraUnavailable):
        sim.start_camera(["main"])


def test_set_frame_provider_updates_cameras_and_supports_camera(
    sim: SimTransport,
) -> None:
    assert sim.supports_camera is False
    assert sim._auth_result["cameras"] == []

    sim.set_frame_provider(_make_provider())
    assert sim.supports_camera is True
    assert sim._auth_result["cameras"] == ["main"]

    sim.set_frame_provider(None)
    assert sim.supports_camera is False
    assert sim._auth_result["cameras"] == []


def test_read_frame_returns_bgr_ndarray_after_start_and_tick(
    sim: SimTransport,
) -> None:
    import numpy as np

    sim.set_frame_provider(_make_provider())
    sim.start_camera(["main"])
    sim.wait_for_update(1.0)  # one pump tick — renders via _tick()

    frame = sim.read_frame()
    assert isinstance(frame, np.ndarray)
    assert frame.shape == (2, 2, 3)
    assert frame.dtype == np.uint8
    assert frame[0, 0].tolist() == [10, 20, 30]  # BGR order preserved


def test_provider_receives_pose_directly_not_via_get_state(sim: SimTransport) -> None:
    """Regression guard: the provider takes (camera, x, y, theta, neck_yaw,
    neck_pitch) as plain floats rather than reading pose back out through
    `get_state()` — wired the way the plan's own sketch of this seam shows
    it, `get_state()` calls `_tick()`, which would call the provider, which
    would call `get_state()` again, recursing forever. Also checks the pose
    passed through is the live one, not a stale snapshot from
    construction."""
    seen = []

    def provider(camera, x, y, theta, neck_yaw, neck_pitch):
        seen.append((x, y, theta))
        return bytes([0, 0, 0] * 1), 1, 1

    sim.set_frame_provider(provider)
    sim.set_camera_preview_open(True)  # render without start_camera()

    sim.wait_for_update(1.0)
    assert seen == [(0.0, 0.0, 0.0)]  # fresh sim, at the origin

    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.5, "angular_z": 0.0})
    start = time.monotonic()
    while time.monotonic() - start < 0.2:
        sim.wait_for_update(1.0)
    sim.send({"type": protocol.CMD_DRIVE, "linear_x": 0.0, "angular_z": 0.0})
    sim.wait_for_update(1.0)

    last_x, _, _ = seen[-1]
    assert last_x > 0.0  # the provider saw the robot actually move


def test_provider_receives_neck_joints_ramped_by_set_neck(sim: SimTransport) -> None:
    """The neck angles the provider aims the camera with come from
    `_joint_positions` (moved by real servo commands — `set_neck`/
    `look_left`/`look_right`/`set_single_servo`), never from `CMD_HEAD_LOOK`
    — that command is a v1 stub on the real robot too
    (controllers/head.py), so a simulator that responded to it would pan
    while a real robot sits still."""
    seen = []

    def provider(camera, x, y, theta, neck_yaw, neck_pitch):
        seen.append((neck_yaw, neck_pitch))
        return bytes([0, 0, 0] * 1), 1, 1

    sim.set_frame_provider(provider)
    sim.set_camera_preview_open(True)
    sim.wait_for_update(1.0)
    assert seen[-1] == (0.0, 0.0)  # fresh sim, neck centred

    # A real servo command: look_left() sends neckYaw=45deg via
    # CMD_SERVO_COMMAND, which SimTransport ramps for real.
    import math

    cmd_id = sim.send(
        {
            "type": protocol.CMD_SERVO_COMMAND,
            "servos": {"neckYaw": math.radians(45)},
            "duration": 0.05,
        }
    )
    sim.wait_for_ack(cmd_id)
    start = time.monotonic()
    while time.monotonic() - start < 0.3:
        sim.wait_for_update(1.0)

    last_yaw, last_pitch = seen[-1]
    assert last_yaw == pytest.approx(math.radians(45), abs=0.05)
    assert last_pitch == pytest.approx(0.0, abs=1e-6)


def test_provider_ignores_head_look_stub(sim: SimTransport) -> None:
    """`CMD_HEAD_LOOK` (`HeadController.look`) must never move the neck
    joints the camera reads — it's a stub on the real robot
    (controllers/head.py) and making the sim respond would be the exact
    sim/robot behaviour divergence the transport seam exists to prevent."""
    seen = []

    def provider(camera, x, y, theta, neck_yaw, neck_pitch):
        seen.append((neck_yaw, neck_pitch))
        return bytes([0, 0, 0] * 1), 1, 1

    sim.set_frame_provider(provider)
    sim.set_camera_preview_open(True)

    cmd_id = sim.send({"type": protocol.CMD_HEAD_LOOK, "pan": 30.0, "tilt": 20.0})
    ack = sim.wait_for_ack(cmd_id)
    assert ack.get("ok") is True  # acks like a real stub handler

    sim.wait_for_update(1.0)
    assert seen[-1] == (0.0, 0.0)  # ...but the neck never actually moved


def test_stop_camera_makes_read_frame_return_none_again(sim: SimTransport) -> None:
    sim.set_frame_provider(_make_provider())
    sim.start_camera(["main"])
    sim.wait_for_update(1.0)
    assert sim.read_frame() is not None

    sim.stop_camera()
    assert sim.read_frame() is None


def test_camera_renders_on_pump_tick_without_read_frame_call(
    sim: SimTransport,
) -> None:
    """The regression test for the dev/SIMULATOR.md §3.6 amendment: rendering is driven by
    the pump (wait_for_update/read_telemetry), not by read_frame() — a
    program that only polls telemetry still gets fresh frames for the
    preview panel."""
    provider = _make_provider()
    sim.set_frame_provider(provider)
    sim.start_camera(["main"])

    sim.wait_for_update(1.0)
    sim.read_telemetry()
    sim.wait_for_update(1.0)

    assert len(provider.calls) == 3  # once per pump tick, zero read_frame() calls


def test_camera_not_rendered_when_not_gated(sim: SimTransport) -> None:
    provider = _make_provider()
    sim.set_frame_provider(provider)
    # Neither start_camera() nor the preview panel has asked for anything.
    sim.wait_for_update(1.0)
    assert provider.calls == []


def test_preview_open_renders_but_read_frame_still_gated_on_start_camera(
    sim: SimTransport,
) -> None:
    provider = _make_provider()
    sim.set_frame_provider(provider)
    sim.set_camera_preview_open(True)
    sim.wait_for_update(1.0)

    assert provider.calls == ["main"]  # rendered for the preview
    assert sim.read_frame() is None  # but start_camera() was never called
