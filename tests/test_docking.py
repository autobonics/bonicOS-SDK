"""Docking (PROTOCOL.md §5.3.1): payloads, ``dock_status`` waiting, and a
no-addon refusal surfacing as ``CommandError``."""

from __future__ import annotations

import pytest

import bonicos
from bonicos import protocol
from bonicos.exceptions import CommandError

NO_ADDON = (
    "docking isn't available on this robot — it needs the docking addon "
    "(a rear camera and an AprilTag dock) — see API.md §4"
)


def test_a_robot_without_the_addon_says_so(robot, transport) -> None:
    for command, call in (
        (protocol.CMD_SAVE_DOCK, robot.nav.save_dock),
        (protocol.CMD_DOCK, robot.nav.dock),
        (protocol.CMD_UNDOCK, robot.nav.undock),
    ):
        transport.script_ack(command, {"ok": False, "error": NO_ADDON})
        with pytest.raises(CommandError) as err:
            call()
        assert err.value.command == command
        assert "docking addon" in err.value.reason


def test_save_dock_sends_the_default_name_and_no_coordinates(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SAVE_DOCK, {"ok": True, "name": "default"})
    assert robot.nav.save_dock() is True
    sent = transport.sent[-1]
    assert sent["name"] == "default"
    assert "x" not in sent and "y" not in sent


def test_dock_waits_for_its_own_dock_status(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DOCK, {"ok": True, "goal_id": "d2"})
    # A previous attempt's result is still cached and must not be matched.
    transport.push_event(
        protocol.EVENT_DOCK_STATUS, {"status": "succeeded", "goal_id": "d1"}
    )
    transport.push_event(
        protocol.EVENT_DOCK_STATUS,
        {"status": "failed", "goal_id": "d2", "error_code": 903},
    )
    assert robot.nav.dock(timeout=1.0) is False
    assert robot.nav.get_dock_status() == "failed"
    assert robot.nav.get_dock_result()["error_code"] == 903


def test_dock_names_the_dock_and_allows_a_slow_ack(robot, transport) -> None:
    # The dock ack may take several seconds.
    seen = {}
    real = transport.wait_for_ack

    def spy(cmd_id, timeout=5.0):
        seen["timeout"] = timeout
        return real(cmd_id, timeout)

    transport.wait_for_ack = spy
    transport.script_ack(protocol.CMD_DOCK, {"ok": True, "goal_id": "d1"})
    assert robot.nav.dock("charger", wait=False) is True
    assert transport.sent[-1]["name"] == "charger"
    assert seen["timeout"] > 8.0


def test_undock_waits_for_success(robot, transport) -> None:
    transport.script_ack(protocol.CMD_UNDOCK, {"ok": True, "goal_id": "u1"})
    transport.push_event(
        protocol.EVENT_DOCK_STATUS, {"status": "succeeded", "goal_id": "u1"}
    )
    assert robot.nav.undock(timeout=1.0) is True


def test_list_docks_extracts_names(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_LIST_DOCKS,
        {"ok": True, "map": "lab",
         "docks": [{"name": "default", "x": 1.0, "y": 2.0, "theta": 3.14}]},
    )
    assert robot.nav.list_docks(map="lab") == ["default"]
    assert transport.sent[-1]["map"] == "lab"
    assert robot.nav.get_docks()[0]["theta"] == 3.14


def test_dock_status_is_cached_by_the_websocket_transport() -> None:
    assert protocol.EVENT_DOCK_STATUS in protocol.ASYNC_EVENTS


def test_dock_status_idle_before_any_attempt(robot) -> None:
    assert robot.nav.get_dock_status() == "idle"
    assert robot.nav.get_dock_result() is None


# --- the simulator refuses docking --------------------------------------------


@pytest.fixture()
def sim_robot():
    bot = bonicos.BonicBot.simulated()
    try:
        yield bot
    finally:
        bonicos.use_transport(None)


def test_sim_refuses_docking_with_a_sentence(sim_robot) -> None:
    with pytest.raises(CommandError, match="isn't available in the simulator"):
        sim_robot.dock()
    with pytest.raises(CommandError, match="docking addon"):
        sim_robot.save_dock()


def test_sim_lists_no_docks(sim_robot) -> None:
    assert sim_robot.list_docks() == []
