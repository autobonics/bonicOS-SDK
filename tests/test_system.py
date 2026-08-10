from __future__ import annotations

import threading
import time

import pytest

from bonicos import protocol
from bonicos.exceptions import FeatureUnavailable

from .conftest import FakeRobot


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
    assert robot.system.restart_base_session() is False


def test_restart_base_session_raises_when_feature_gated(transport) -> None:
    robot = FakeRobot(transport, features={"session_control": False})
    with pytest.raises(FeatureUnavailable):
        robot.system.restart_base_session()
    assert transport.sent == []


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


def test_trigger_update(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_TRIGGER_UPDATE, {"ok": True, "detail": "restarting"}
    )
    assert robot.system.trigger_update() is True


def test_speak(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SPEAK, {"ok": True})
    assert robot.system.speak("hello there", voice="default") is True
    sent = transport.sent[-1]
    assert sent["text"] == "hello there"
    assert sent["voice"] == "default"


def test_ask_llm_streams_and_joins_tokens(robot, transport) -> None:
    def pusher() -> None:
        time.sleep(0.02)
        transport.push_event(
            protocol.EVENT_LLM_TOKEN, {"id": 1, "token": "Hel", "done": False}
        )
        time.sleep(0.02)
        transport.push_event(
            protocol.EVENT_LLM_TOKEN, {"id": 1, "token": "lo", "done": True}
        )

    threading.Thread(target=pusher, daemon=True).start()
    text = robot.system.ask_llm("say hi", timeout=2.0)
    assert text == "Hello"
    assert transport.sent[-1]["type"] == protocol.CMD_LLM_QUERY
    assert transport.sent[-1]["prompt"] == "say hi"


def test_ask_llm_times_out_without_a_done_token(robot, transport) -> None:
    assert robot.system.ask_llm("say hi", timeout=0.1) == ""
