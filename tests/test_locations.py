"""Named locations (PROTOCOL.md §5.3) — live since 2026-08-31.

Two things these tests pin down, both of which the SDK got wrong while the
server-side handlers were stubs:

  - a location is a MAP-scoped pose, so `map` has to reach the wire when the
    caller names one and stay off it when they don't (the server falls back to
    the active session's map on absence, not on null);
  - `list_locations` gets dicts back, not the bare strings the stub returned.
"""

from __future__ import annotations

import pytest

from bonicos import protocol
from bonicos.exceptions import CommandError


def test_save_location_here_omits_coordinates(robot, transport) -> None:
    """No x/y means "save where the robot is" — the server reads its own
    pose, and sending a 0.0 placeholder would silently save the map origin."""
    transport.script_ack(protocol.CMD_SAVE_LOCATION, {"ok": True, "name": "kitchen"})
    assert robot.nav.save_location("kitchen") is True
    sent = transport.sent[-1]
    assert sent["name"] == "kitchen"
    assert "x" not in sent and "y" not in sent
    assert "map" not in sent


def test_save_location_with_coordinates_sends_the_pose(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SAVE_LOCATION, {"ok": True, "name": "desk"})
    assert robot.nav.save_location("desk", x=1.5, y=-2.0, theta=0.75) is True
    sent = transport.sent[-1]
    assert (sent["x"], sent["y"], sent["theta"]) == (1.5, -2.0, 0.75)


def test_map_is_sent_only_when_named(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DELETE_LOCATION, {"ok": True})
    robot.nav.delete_location("desk")
    assert "map" not in transport.sent[-1]

    transport.script_ack(protocol.CMD_DELETE_LOCATION, {"ok": True})
    robot.nav.delete_location("desk", map="lab")
    assert transport.sent[-1]["map"] == "lab"


def test_list_locations_extracts_names_from_records(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_LIST_LOCATIONS,
        {
            "ok": True,
            "map": "lab",
            "locations": [
                {"name": "desk", "x": 1.0, "y": 2.0, "theta": 0.0},
                {"name": "door", "x": 3.0, "y": 4.0, "theta": 1.5},
            ],
        },
    )
    assert robot.nav.list_locations() == ["desk", "door"]


def test_get_locations_returns_full_records(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_LIST_LOCATIONS,
        {"ok": True, "locations": [{"name": "desk", "x": 1.0, "y": 2.0, "theta": 0.5}]},
    )
    assert robot.nav.get_locations() == [
        {"name": "desk", "x": 1.0, "y": 2.0, "theta": 0.5}
    ]


def test_list_locations_tolerates_the_old_stub_shape(robot, transport) -> None:
    """An older robot_app returned bare names. Degrade rather than raise."""
    transport.script_ack(
        protocol.CMD_LIST_LOCATIONS, {"ok": True, "locations": ["desk", "door"]}
    )
    assert robot.nav.list_locations() == ["desk", "door"]


def test_goto_location_refused_does_not_wait_for_a_goal(robot, transport) -> None:
    """A refusal (no such location, wrong map) is not a goal to wait on —
    returning True here would have the caller believe the robot is driving."""
    transport.script_ack(
        protocol.CMD_GOTO_LOCATION,
        {"ok": False, "name": "nowhere", "error": "no location 'nowhere' saved"},
    )
    with pytest.raises(CommandError, match="no location 'nowhere' saved") as err:
        robot.nav.goto_location("nowhere", timeout=0.1)
    assert err.value.command == protocol.CMD_GOTO_LOCATION


def test_goto_location_waits_on_the_returned_goal_id(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_GOTO_LOCATION, {"ok": True, "name": "desk", "goal_id": "7"}
    )
    transport.set_telemetry("nav_status", {"status": "succeeded", "goal_id": "7"})
    assert robot.nav.goto_location("desk", timeout=1.0) is True


# --- against the sim, which now implements these for real -------------------


@pytest.fixture()
def sim_robot():
    import bonicos

    bot = bonicos.BonicBot.simulated()
    try:
        yield bot
    finally:
        # Don't leak the sticky registration to other tests.
        bonicos.use_transport(None)


def _navigating(bot, name="lab"):
    bot.nav.enter_mapping_mode()
    bot.nav.save_map(name)
    bot.nav.enter_navigation_mode(name)


def test_sim_requires_a_map_before_saving(sim_robot) -> None:
    # Idle: there is no map for a map-frame pose to belong to.
    with pytest.raises(CommandError, match="locations belong to a map"):
        sim_robot.nav.save_location("kitchen")
    assert sim_robot.nav.list_locations() == []


def test_sim_round_trips_a_saved_location(sim_robot) -> None:
    _navigating(sim_robot)
    assert sim_robot.nav.save_location("desk", x=1.0, y=0.5, theta=0.0) is True
    assert sim_robot.nav.list_locations() == ["desk"]
    record = sim_robot.nav.get_locations()[0]
    assert (record["x"], record["y"]) == (1.0, 0.5)


def test_sim_goto_location_actually_drives(sim_robot) -> None:
    _navigating(sim_robot)
    sim_robot.nav.save_location("desk", x=1.0, y=0.0)
    assert sim_robot.nav.goto_location("desk", timeout=60.0) is True
    position = sim_robot.sensors.get_position()
    assert abs(position["x"] - 1.0) < 0.25


def test_sim_locations_die_with_their_map(sim_robot) -> None:
    _navigating(sim_robot, "lab")
    sim_robot.nav.save_location("desk", x=1.0, y=0.0)
    sim_robot.nav.save_map("other")
    sim_robot.nav.enter_navigation_mode("other")
    sim_robot.nav.delete_map("lab")
    # "lab" is gone, so its places are too — a later map reusing the name must
    # not inherit places from a different room.
    assert sim_robot.nav.list_locations(map="lab") == []


def test_sim_refuses_a_location_from_another_map(sim_robot) -> None:
    _navigating(sim_robot, "lab")
    sim_robot.nav.save_location("desk", x=1.0, y=0.0)
    sim_robot.nav.save_map("other")
    sim_robot.nav.enter_navigation_mode("other")
    # Navigating on "other" — a pose from "lab" is a well-formed coordinate
    # pointing at a different room, so driving there would be wrong.
    with pytest.raises(CommandError, match="navigating on 'other', not 'lab'"):
        sim_robot.nav.goto_location("desk", map="lab", timeout=1.0)
