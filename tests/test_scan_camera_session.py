"""The cost-control and session commands added to robot_app in the
unify-robot-series merge: on-demand `scan`, per-viewer camera idling, and the
two halves of base-session supervision.
"""

from __future__ import annotations

import math

from bonicos import protocol

# --- laser scan (on demand) -------------------------------------------------


def test_get_scan_enables_the_stream_once(robot, transport) -> None:
    """Scan is the one telemetry topic that is off by default, so reading it
    has to ask for it — but only once, not on every poll."""
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True, "enabled": True})
    assert robot.sensors.get_scan() is None  # nothing has arrived yet
    sent = [m for m in transport.sent if m["type"] == protocol.CMD_SET_SCAN_ENABLED]
    assert len(sent) == 1 and sent[0]["enabled"] is True

    robot.sensors.get_scan()
    robot.sensors.get_scan()
    still = [m for m in transport.sent if m["type"] == protocol.CMD_SET_SCAN_ENABLED]
    assert len(still) == 1


def test_scan_is_cached_telemetry(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True})
    transport.set_telemetry(
        "scan",
        {
            "origin": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "angle_min": 0.0,
            "angle_increment": math.pi / 2,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [1.0, None, 2.0],
        },
    )
    scan = robot.sensors.get_scan()
    assert scan["ranges"] == [1.0, None, 2.0]
    assert scan["range_max"] == 10.0


def test_scan_points_are_map_frame_and_skip_no_returns(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True})
    transport.set_telemetry(
        "scan",
        {
            # Scanner sitting at (10, 20), facing +x.
            "origin": {"x": 10.0, "y": 20.0, "theta": 0.0},
            "angle_min": 0.0,
            "angle_increment": math.pi / 2,  # 0 rad, then 90 rad
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [2.0, None, 3.0],  # ahead, no return, then behind
        },
    )
    points = robot.sensors.get_scan_points()
    assert len(points) == 2  # the None is dropped, not rendered at the origin
    # First ray: straight ahead of a scanner at (10, 20).
    assert math.isclose(points[0][0], 12.0, abs_tol=1e-9)
    assert math.isclose(points[0][1], 20.0, abs_tol=1e-9)
    # Third ray is two increments round, i.e. pointing back down -x.
    assert math.isclose(points[1][0], 7.0, abs_tol=1e-9)


def test_scan_points_account_for_scanner_heading(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True})
    transport.set_telemetry(
        "scan",
        {
            # Same ray, but the robot has turned 90 deg — the point must move.
            "origin": {"x": 0.0, "y": 0.0, "theta": math.pi / 2},
            "angle_min": 0.0,
            "angle_increment": 0.1,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [2.0],
        },
    )
    ((x, y),) = robot.sensors.get_scan_points()
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 2.0, abs_tol=1e-9)


def test_scan_points_drop_out_of_band_readings(robot, transport) -> None:
    """Below range_min or above range_max is not a measurement. Keeping them
    draws a ring of noise at the sensor's limit that looks like a wall —
    matches the frontend's ScanPoints filter."""
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True})
    transport.set_telemetry(
        "scan",
        {
            "origin": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "angle_min": 0.0,
            "angle_increment": 0.1,
            "range_min": 0.5,
            "range_max": 8.0,
            #        too near, valid, too far, no return
            "ranges": [0.2, 3.0, 12.0, None],
        },
    )
    points = robot.sensors.get_scan_points()
    assert len(points) == 1
    assert math.isclose(points[0][0], 3.0 * math.cos(0.1), abs_tol=1e-9)


def test_scan_can_be_turned_off_explicitly(robot, transport) -> None:
    transport.script_ack(protocol.CMD_SET_SCAN_ENABLED, {"ok": True, "enabled": False})
    assert robot.sensors.set_scan_enabled(False) is True
    assert transport.sent[-1]["enabled"] is False


# --- camera idling ----------------------------------------------------------


def test_pause_and_resume_camera(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_SET_CAMERA_ENABLED, {"ok": True, "enabled": False, "cameras": []}
    )
    assert robot.camera.pause() is True
    sent = transport.sent[-1]
    assert sent["enabled"] is False
    assert "camera" not in sent  # omitted = all of this peer's tracks

    transport.script_ack(protocol.CMD_SET_CAMERA_ENABLED, {"ok": True, "enabled": True})
    assert robot.camera.resume("face") is True
    sent = transport.sent[-1]
    assert sent["enabled"] is True
    assert sent["camera"] == "face"


def test_pause_reports_false_on_a_lane_with_no_video(robot, transport) -> None:
    """A local-WebSocket or BLE connection carries no media tracks. Saying
    "ok" would let a caller believe they'd stopped paying for video."""
    transport.script_ack(
        protocol.CMD_SET_CAMERA_ENABLED,
        {
            "ok": False,
            "error": "this connection carries no camera tracks",
            "cameras": [],
        },
    )
    assert robot.camera.pause() is False


# --- base session -----------------------------------------------------------


def test_start_and_stop_base_session(robot, transport) -> None:
    transport.script_ack(protocol.CMD_START_BASE_SESSION, {"ok": True, "running": True})
    assert robot.system.start_base_session() is True
    assert transport.sent[-1]["type"] == protocol.CMD_START_BASE_SESSION

    transport.script_ack(protocol.CMD_STOP_BASE_SESSION, {"ok": True, "running": False})
    assert robot.system.stop_base_session() is True


def test_stop_base_session_refused_while_moving(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_STOP_BASE_SESSION,
        {"ok": False, "error": "robot is moving — stop it first"},
    )
    assert robot.system.stop_base_session() is False
