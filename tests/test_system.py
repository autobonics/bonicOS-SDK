from __future__ import annotations

import pytest

from bonicos import BonicBot, protocol
from bonicos.exceptions import CommandError


def test_health(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEALTH, {"cpu": 12.0, "ram": 30.0})
    result = robot.system.health()
    assert result["cpu"] == 12.0


def test_restart_base_session(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_RESTART_BASE_SESSION,
        {"ok": True, "error": None, "running": True, "transitioning": False},
    )
    assert robot.system.restart_base_session() is True


def test_restart_base_session_refused_while_moving(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_RESTART_BASE_SESSION,
        {
            "ok": False,
            "error": "robot is moving — stop it first",
            "running": True,
            "transitioning": False,
        },
    )
    with pytest.raises(CommandError, match="robot is moving"):
        robot.system.restart_base_session()


def test_restart_base_session_raises_when_robot_reports_no_session_control(
    robot, transport
) -> None:
    transport.script_error(
        protocol.CMD_RESTART_BASE_SESSION,
        "this robot has no supervised ROS stack to restart",
    )
    with pytest.raises(CommandError, match="no supervised ROS stack"):
        robot.system.restart_base_session()


def test_get_session_status(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_GET_SESSION_STATUS,
        {
            "base": {
                "running": True,
                "owned": True,
                "transitioning": False,
                "error": None,
            },
            "nav": {
                "mode": "navigating",
                "map": "office",
                "transitioning": False,
                "localized": True,
            },
            "health": {
                "running": True,
                "owned": True,
                "clock_publishers": 1,
                "issues": [],
            },
        },
    )
    status = robot.system.get_session_status()
    assert status["base"]["running"] is True
    assert status["nav"]["mode"] == "navigating"
    assert status["health"]["issues"] == []


def test_get_base_session_reads_cached_telemetry(robot, transport) -> None:
    assert robot.system.get_base_session() is None
    transport.set_telemetry(
        "base_session",
        {"running": True, "owned": False, "transitioning": False, "error": None},
    )
    assert robot.system.get_base_session() == {
        "type": "base_session",
        "running": True,
        "owned": False,
        "transitioning": False,
        "error": None,
    }


def test_get_session_health_reads_cached_telemetry(robot, transport) -> None:
    assert robot.system.get_session_health() is None
    transport.set_telemetry(
        "session_health",
        {"ok": False, "base": {}, "nav": {}, "issues": ["amcl_not_running"]},
    )
    health = robot.system.get_session_health()
    assert health["ok"] is False
    assert health["issues"] == ["amcl_not_running"]


def test_reconfig_wifi(robot, transport) -> None:
    transport.script_ack(protocol.CMD_RECONFIG_WIFI, {"ok": True})
    assert robot.system.reconfig_wifi("myssid", "mypassword") is True
    sent = transport.sent[-1]
    assert sent["ssid"] == "myssid"
    assert sent["password"] == "mypassword"


def test_update_status_reads_through_to_the_host(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_UPDATE_STATUS,
        {"ok": True, "state": "idle", "reported_version": "0.2.0",
         "last_update": {"version": "0.3.0", "result": "rolled_back"}},
    )
    status = robot.system.update_status()
    # The read that survives the restart an install causes: how it ended is
    # only knowable after reconnecting to whatever version came up.
    assert status["last_update"]["result"] == "rolled_back"


def test_shutdown_halts_the_machine(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SHUTDOWN, {"ok": True})
    assert robot.system.shutdown() is True
    assert transport.sent[-1]["type"] == protocol.CMD_SHUTDOWN


def test_shutdown_reports_a_refusal(robot, transport) -> None:
    # The refusal matters more than most: a program that believes the robot is
    # off walks away from one that is still powered.
    transport.script_ack(
        protocol.CMD_SHUTDOWN, {"ok": False, "error": "poweroff not permitted"}
    )
    with pytest.raises(CommandError, match="poweroff not permitted"):
        robot.system.shutdown()


def test_get_update_status_reads_cached_telemetry(robot, transport) -> None:
    assert robot.system.get_update_status() is None
    transport.set_telemetry(
        protocol.EVENT_UPDATE_PROGRESS,
        {"state": "installing", "phase": "pulling", "percent": 40,
         "version": "0.3.0"},
    )
    progress = robot.system.get_update_status()
    assert progress is not None
    assert progress["percent"] == 40


def test_speak_sends_only_what_was_given(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SPEAK, {"ok": True})
    assert robot.system.speak("hello there") is True
    assert transport.sent[-1] == {"type": protocol.CMD_SPEAK, "text": "hello there",
                                  "id": transport.sent[-1]["id"]}


def test_speak_forwards_every_option(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SPEAK, {"ok": True})
    # Through the BonicBot facade, so its forwarding is covered too.
    assert BonicBot.speak(robot, "namaste", "Zephyr", language="hi-IN",
                          rate=0.9, engine="cloud") is True
    sent = transport.sent[-1]
    assert (sent["text"], sent["voice"], sent["language"], sent["rate"],
            sent["engine"]) == ("namaste", "Zephyr", "hi-IN", 0.9, "cloud")
    assert "agent_id" not in sent


def test_speak_forwards_agent_id(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SPEAK, {"ok": True})
    assert BonicBot.speak(robot, "welcome", agent_id="agent-42") is True
    assert transport.sent[-1]["agent_id"] == "agent-42"


def test_speak_raises_the_robots_reason(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SPEAK, {
        "ok": False, "error": "cloud voices need BonicOS, which this robot "
                              "doesn't have"})
    with pytest.raises(CommandError, match="cloud voices need BonicOS"):
        robot.system.speak("hi", engine="cloud")
