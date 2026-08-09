from __future__ import annotations

import base64
import zlib

import pytest

from bonicos import protocol
from bonicos.exceptions import FeatureUnavailable

from .conftest import FakeRobot


def test_go_to_waits_for_terminal_nav_status(robot, transport) -> None:
    transport.script_ack(protocol.CMD_NAV_GOAL, {"goal_id": "g1"})
    transport.push_event(
        protocol.EVENT_NAV_STATUS, {"status": "succeeded", "goal_id": "g1"}
    )
    assert robot.nav.go_to(1.0, 2.0) is True


def test_go_to_no_wait_returns_immediately(robot, transport) -> None:
    transport.script_ack(protocol.CMD_NAV_GOAL, {"goal_id": "g1"})
    assert robot.nav.go_to(1.0, 2.0, wait=False) is True


def test_go_to_failed_status_returns_false(robot, transport) -> None:
    transport.script_ack(protocol.CMD_NAV_GOAL, {"goal_id": "g1"})
    transport.push_event(
        protocol.EVENT_NAV_STATUS, {"status": "failed", "goal_id": "g1"}
    )
    assert robot.nav.go_to(1.0, 2.0) is False


def test_go_to_raises_when_feature_gated(transport) -> None:
    robot = FakeRobot(transport, features={"navigation": False})
    with pytest.raises(FeatureUnavailable):
        robot.nav.go_to(1.0, 2.0)
    assert transport.sent == []  # never sent — SDK-side fast fail


def test_wait_for_goal_times_out_without_terminal_status(robot, transport) -> None:
    assert robot.nav.wait_for_goal(timeout=0.2) is False


def test_navigate_waypoints_builds_payload(robot, transport) -> None:
    transport.script_ack(protocol.CMD_NAVIGATE_THROUGH_WAYPOINTS, {"goal_id": "g2"})
    robot.nav.navigate_waypoints([(1.0, 2.0), (3.0, 4.0, 1.57)], wait=False)
    waypoints = transport.sent[-1]["waypoints"]
    assert waypoints == [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0, "theta": 1.57}]


def test_cancel_goal(robot, transport) -> None:
    transport.script_ack(protocol.CMD_CANCEL_NAV, {"canceled": True})
    assert robot.nav.cancel_goal() is True


def test_mapping_and_maps(robot, transport) -> None:
    transport.script_ack(protocol.CMD_START_MAPPING, {"ok": True})
    transport.script_ack(protocol.CMD_SAVE_MAP, {"ok": True, "name": "office"})
    transport.script_ack(protocol.CMD_LIST_MAPS, {"maps": ["office", "lab"]})
    assert robot.nav.start_mapping() is True
    assert robot.nav.save_map("office") is True
    assert robot.nav.list_maps() == ["office", "lab"]


def test_list_maps_extracts_names_from_server_metadata_dicts(robot, transport) -> None:
    # The real server (bonicOS-robot-app's MapManager.list()) returns
    # metadata dicts, not plain strings — verified against the real M1 sim
    # 2026-08-04. list_maps() must still honor its List[str] contract.
    transport.script_ack(
        protocol.CMD_LIST_MAPS,
        {
            "maps": [
                {"name": "office", "size": 237714, "modified": 1785862027},
                {"name": "lab", "size": 100, "modified": 1785862000},
            ]
        },
    )
    assert robot.nav.list_maps() == ["office", "lab"]


def test_get_map_decodes_compressed_grid(robot, transport) -> None:
    raw = bytes([0, 100, 0, 100] * 4)
    data_b64 = base64.b64encode(zlib.compress(raw)).decode()
    transport.set_telemetry(
        "map", {"info": {"width": 4, "height": 4}, "data_b64": data_b64}
    )
    grid = robot.nav.get_map()
    assert grid is not None
    assert grid["info"]["width"] == 4
    assert grid["data"] == raw


def test_get_map_none_without_telemetry(robot, transport) -> None:
    assert robot.nav.get_map() is None


def test_enter_mapping_mode(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_ENTER_MAPPING_MODE, {"ok": True, "mode": "mapping"}
    )
    assert robot.nav.enter_mapping_mode() is True


def test_enter_mapping_mode_raises_when_feature_gated(transport) -> None:
    robot = FakeRobot(transport, features={"mapping": False})
    with pytest.raises(FeatureUnavailable):
        robot.nav.enter_mapping_mode()
    assert transport.sent == []


def test_enter_navigation_mode_builds_payload_and_reports_failure(
    robot, transport
) -> None:
    transport.script_ack(
        protocol.CMD_ENTER_NAVIGATION_MODE,
        {"ok": False, "mode": "idle", "map": None, "error": "map 'office' not found"},
    )
    assert robot.nav.enter_navigation_mode("office") is False
    assert transport.sent[-1]["name"] == "office"


def test_enter_navigation_mode_raises_when_feature_gated(transport) -> None:
    robot = FakeRobot(transport, features={"navigation": False})
    with pytest.raises(FeatureUnavailable):
        robot.nav.enter_navigation_mode("office")
    assert transport.sent == []


def test_stop_nav_mode(robot, transport) -> None:
    transport.script_ack(protocol.CMD_STOP_NAV_MODE, {"ok": True, "mode": "idle"})
    assert robot.nav.stop_nav_mode() is True


def test_get_nav_mode_issues_fresh_query(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_GET_NAV_MODE,
        {"mode": "navigating", "map": "office", "transitioning": False,
         "localized": True},
    )
    assert robot.nav.get_nav_mode() == {
        "mode": "navigating",
        "map": "office",
        "transitioning": False,
        "localized": True,
    }


def test_get_nav_mode_defaults_when_fields_missing(robot, transport) -> None:
    transport.script_ack(protocol.CMD_GET_NAV_MODE, {})
    assert robot.nav.get_nav_mode() == {
        "mode": "idle",
        "map": None,
        "transitioning": False,
        "localized": False,
    }


def test_delete_map(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DELETE_MAP, {"ok": True, "name": "office"})
    assert robot.nav.delete_map("office") is True
    assert transport.sent[-1]["name"] == "office"


def test_delete_map_refused_for_map_in_use(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_DELETE_MAP,
        {"ok": False, "name": "office", "error": "map is in use"},
    )
    assert robot.nav.delete_map("office") is False


def test_named_locations_are_stubs_that_still_ack(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SAVE_LOCATION, {"ok": True, "name": "kitchen"})
    transport.script_ack(protocol.CMD_LIST_LOCATIONS, {"locations": []})
    assert robot.nav.save_location("kitchen") is True
    assert robot.nav.list_locations() == []
