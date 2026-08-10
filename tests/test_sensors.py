from __future__ import annotations

import math

from bonicos import protocol


def test_get_position_defaults_to_origin(robot, transport) -> None:
    assert robot.sensors.get_position() == {"x": 0.0, "y": 0.0, "theta": 0.0}


def test_get_position_and_heading(robot, transport) -> None:
    transport.set_telemetry("pose", {"x": 1.5, "y": -2.0, "theta": math.pi / 2})
    assert robot.sensors.get_position() == {"x": 1.5, "y": -2.0, "theta": math.pi / 2}
    assert robot.sensors.get_x() == 1.5
    assert robot.sensors.get_y() == -2.0
    assert math.isclose(robot.sensors.get_heading(), 90.0)


def test_get_battery(robot, transport) -> None:
    assert robot.sensors.get_battery() == 0.0
    transport.set_telemetry("battery", {"voltage": 12.0, "current": 1.0, "soc": 73.5})
    assert robot.sensors.get_battery() == 73.5


def test_get_imu(robot, transport) -> None:
    transport.set_telemetry(
        "imu", {"ax": 1, "ay": 2, "az": 3, "gx": 4, "gy": 5, "gz": 6}
    )
    assert robot.sensors.get_imu() == {
        "ax": 1,
        "ay": 2,
        "az": 3,
        "gx": 4,
        "gy": 5,
        "gz": 6,
    }


def test_get_distance_traveled_from_explicit_start(robot, transport) -> None:
    transport.set_telemetry(
        "odom", {"x": 3.0, "y": 4.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )
    assert robot.sensors.get_distance_traveled(start=(0.0, 0.0)) == 5.0


def test_get_distance_traveled_defaults_to_first_seen_odom(robot, transport) -> None:
    transport.set_telemetry(
        "odom", {"x": 1.0, "y": 1.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )
    assert robot.sensors.get_distance_traveled() == 0.0  # baseline captured here
    transport.set_telemetry(
        "odom", {"x": 4.0, "y": 5.0, "theta": 0.0, "vx": 0, "vtheta": 0}
    )
    assert robot.sensors.get_distance_traveled() == 5.0


def test_wait_for_data(robot, transport) -> None:
    assert robot.sensors.wait_for_data(timeout=0.1) is False
    transport.set_telemetry("battery", {"soc": 50.0})
    assert robot.sensors.wait_for_data(timeout=0.1) is True


def test_subscribe_sends_event_list(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_SUBSCRIBE, {"ok": True, "events": ["pose", "battery"]}
    )
    assert robot.sensors.subscribe(["pose", "battery"]) is True
    assert transport.sent[-1]["events"] == ["pose", "battery"]
