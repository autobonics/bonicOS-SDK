"""A refusal — an ``error`` reply or an ``ack`` with ``ok: false``
(PROTOCOL.md §2) — raises ``CommandError`` with the robot's reason."""

from __future__ import annotations

import pytest

import bonicos
from bonicos import protocol
from bonicos.exceptions import CommandError


def test_an_ok_false_ack_raises_with_the_robots_error(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_START_MAPPING,
        {"ok": False, "error": "slam_toolbox is not running"},
    )
    with pytest.raises(CommandError) as err:
        robot.nav.start_mapping()
    assert err.value.command == protocol.CMD_START_MAPPING
    assert err.value.reason == "slam_toolbox is not running"
    assert str(err.value) == "start_mapping: slam_toolbox is not running"


def test_the_whole_reply_rides_on_the_error(robot, transport) -> None:
    # Extra fields in the reply — here, the known names — ride on the error.
    transport.script_ack(
        protocol.CMD_DISPLAY_ANIMATION,
        {"ok": False, "error": "unknown animation: 'wave'", "known": ["happy", "sad"]},
    )
    with pytest.raises(CommandError) as err:
        robot.head.set_display_animation("wave")
    assert err.value.result["known"] == ["happy", "sad"]


def test_detail_is_read_when_there_is_no_error_field(robot, transport) -> None:
    # `detail` is used when there is no `error`.
    transport.script_ack(
        protocol.CMD_SPEAK, {"ok": False, "detail": "no speaker on this robot"}
    )
    with pytest.raises(CommandError, match="no speaker on this robot"):
        robot.system.speak("hi")


def test_a_refusal_with_no_reason_still_raises(robot, transport) -> None:
    # ok:false with no reason still raises.
    transport.script_ack(protocol.CMD_SAVE_MAP, {"ok": False, "name": "lab"})
    with pytest.raises(CommandError, match="without saying why"):
        robot.nav.save_map("lab")


def test_an_ack_without_ok_is_success(robot, transport) -> None:
    # nav_goal answers {goal_id} and list_maps {maps}; neither says `ok`.
    transport.script_ack(protocol.CMD_LIST_MAPS, {"maps": ["lab"]})
    assert robot.nav.list_maps() == ["lab"]


def test_update_status_unavailable_is_an_answer_not_an_error(robot, transport) -> None:
    # state "unavailable" is returned, not raised.
    transport.script_ack(
        protocol.CMD_UPDATE_STATUS,
        {"ok": False, "state": "unavailable", "error": "no bonic-host"},
    )
    assert robot.system.update_status()["state"] == "unavailable"


def test_update_status_other_failures_still_raise(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_UPDATE_STATUS, {"ok": False, "error": "host answered 500"}
    )
    with pytest.raises(CommandError, match="host answered 500"):
        robot.system.update_status()


def test_reconfig_wifi_waits_long_enough_for_the_join(robot, transport) -> None:
    # The join can take up to ~50 s.
    seen = {}
    real = transport.wait_for_ack

    def spy(cmd_id, timeout=5.0):
        seen["timeout"] = timeout
        return real(cmd_id, timeout)

    transport.wait_for_ack = spy
    robot.system.reconfig_wifi("home", "secret")
    assert seen["timeout"] >= 50.0


# --- simulator refusals -------------------------------------------------------


@pytest.fixture()
def sim_robot():
    bot = bonicos.BonicBot.simulated()
    try:
        yield bot
    finally:
        bonicos.use_transport(None)


def test_sim_map_refusals_carry_a_reason(sim_robot) -> None:
    with pytest.raises(CommandError, match="map not found"):
        sim_robot.nav.load_map("nowhere")
    with pytest.raises(CommandError, match="map not found"):
        sim_robot.nav.delete_map("nowhere")


# --- a simulated robot with fewer joints -------------------------------------


@pytest.fixture()
def small_sim_robot():
    bot = bonicos.BonicBot.simulated(joints=["leftElbow", "neckYaw"])
    try:
        yield bot
    finally:
        bonicos.use_transport(None)


def test_sim_joint_not_fitted_raises_like_a_robot(small_sim_robot) -> None:
    with pytest.raises(CommandError, match="this robot has no leftGripper, rightGripper"):
        small_sim_robot.open_grippers()


def test_sim_partly_fitted_moves_the_rest_and_warns(small_sim_robot) -> None:
    with pytest.warns(UserWarning, match="no leftWristYaw"):
        assert small_sim_robot.set_servos(
            {"leftElbow": 20.0, "leftWristYaw": 10.0}, timeout=5.0
        ) is True
    # set_servos returns once within its convergence tolerance.
    tolerance = small_sim_robot.arm.CONVERGENCE_TOLERANCE_DEG
    assert small_sim_robot.get_servo_angles()["leftElbow"] == pytest.approx(20.0, abs=tolerance)


def test_sim_misspelled_joint_raises(small_sim_robot) -> None:
    with pytest.raises(CommandError, match="no joint called leftElbw"):
        small_sim_robot.set_servos({"leftElbw": 10.0})
