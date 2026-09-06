"""Head expression & LED matrix (API.md §6).

Live on A series since robot_app gained the face-matrix path. These pin the
two things that were wrong or absent while it was a stub: the unit conversion
on ``look``, and telling the caller when the robot did not actually do what
was asked.
"""

from __future__ import annotations

import math
import warnings

import pytest

from bonicos import protocol
from bonicos.enums import DisplayAnimation, HeadMode


def test_set_expression_sends_the_mode(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEAD_MODE, {"ok": True, "mode": "happy"})
    assert robot.head.set_expression(HeadMode.HAPPY) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_HEAD_MODE
    assert sent["mode"] == "happy"


def test_set_expression_accepts_a_bare_string(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEAD_MODE, {"ok": True})
    assert robot.head.set_expression("angry") is True
    assert transport.sent[-1]["mode"] == "angry"


def test_substituted_expression_warns_rather_than_passing_silently(
    robot, transport
) -> None:
    # "surprised" is a heart in firmware. Returning a bare True here is how a
    # lesson gets built around a face the robot cannot make.
    transport.script_ack(
        protocol.CMD_HEAD_MODE,
        {"ok": True, "mode": "surprised", "substituted": "love (a heart)"},
    )
    with pytest.warns(UserWarning, match="no face in firmware"):
        assert robot.head.set_expression("surprised") is True


def test_a_real_expression_does_not_warn(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEAD_MODE, {"ok": True, "mode": "sad"})
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert robot.head.set_expression("sad") is True


def test_look_converts_degrees_to_radians(robot, transport) -> None:
    # The bug this file exists for: head.look used to forward its arguments
    # unconverted, so look(pan=30) asked the robot for 30 RADIANS while
    # API.md §5 promised degrees at the boundary.
    transport.script_ack(protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": []})
    assert robot.head.look(pan=30.0) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_HEAD_LOOK
    assert math.isclose(sent["pan"], math.radians(30.0))


def test_look_sends_only_the_axes_given(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": []})
    robot.head.look(tilt=-15.0)
    sent = transport.sent[-1]
    assert "pan" not in sent
    assert math.isclose(sent["tilt"], math.radians(-15.0))


def test_look_passes_duration_and_defaults_it(robot, transport) -> None:
    transport.script_ack(protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": []})
    robot.head.look(pan=0.0)
    assert transport.sent[-1]["duration"] == 1.0
    robot.head.look(pan=0.0, duration=2.5)
    assert transport.sent[-1]["duration"] == 2.5


def test_look_keeps_speed_positional_for_older_callers(robot, transport) -> None:
    # look(pan, tilt, speed) was the published signature; `duration` went in
    # keyword-only so this still means what it used to.
    transport.script_ack(protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": []})
    robot.head.look(10.0, None, 30.0)
    sent = transport.sent[-1]
    assert math.isclose(sent["pan"], math.radians(10.0))
    assert sent["speed"] == 30.0


def test_look_is_false_when_the_robot_drove_nothing_asked_for(
    robot, transport
) -> None:
    # tilt alone on an A2: it fits no neck pitch, so nothing moved. Reporting
    # True is how a caller concludes the neck is broken rather than absent.
    transport.script_ack(
        protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": ["neck_pitch_joint"]}
    )
    assert robot.head.look(tilt=20.0) is False


def test_look_is_true_when_some_axis_moved(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": ["neck_pitch_joint"]}
    )
    assert robot.head.look(pan=10.0, tilt=20.0) is True


def test_display_commands_send_their_payloads(robot, transport) -> None:
    for cmd in (
        protocol.CMD_DISPLAY_TEXT,
        protocol.CMD_DISPLAY_COLOR,
        protocol.CMD_DISPLAY_ANIMATION,
        protocol.CMD_DISPLAY_BRIGHTNESS,
        protocol.CMD_DISPLAY_CLEAR,
    ):
        transport.script_ack(cmd, {"ok": True})

    assert robot.head.set_display_text("hi") is True
    assert transport.sent[-1]["text"] == "hi"

    assert robot.head.set_display_color(255, 0, 8) is True
    assert (transport.sent[-1]["r"], transport.sent[-1]["b"]) == (255, 8)

    assert robot.head.set_display_brightness(64) is True
    assert transport.sent[-1]["value"] == 64

    assert robot.head.clear_display() is True
    assert transport.sent[-1]["type"] == protocol.CMD_DISPLAY_CLEAR


def test_play_and_pause_ride_the_animation_command(robot, transport) -> None:
    # bonicos spells these as set_display_animation("play"/"pause"); the server
    # turns them into their own matrix actions.
    transport.script_ack(protocol.CMD_DISPLAY_ANIMATION, {"ok": True})
    assert robot.head.play_display() is True
    assert transport.sent[-1]["mode"] == "play"
    assert robot.head.pause_display() is True
    assert transport.sent[-1]["mode"] == "pause"


def test_animation_accepts_a_raw_firmware_index(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_ANIMATION, {"ok": True})
    assert robot.head.set_display_animation(7) is True
    assert transport.sent[-1]["mode"] == 7


def test_animation_accepts_the_enum_and_sends_its_name(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_ANIMATION, {"ok": True})
    assert robot.head.set_display_animation(DisplayAnimation.RAINBOW_WAVE) is True
    assert transport.sent[-1]["mode"] == "rainbow_wave"


def test_animation_enum_mirrors_the_robot_side_table(robot) -> None:
    # These names are resolved by robot_app against its own ANIMATION_MODES
    # (core/led_matrix.py), which mirrors the firmware enum. A member whose
    # value drifts from that table is a name the robot will simply refuse.
    assert {member.value for member in DisplayAnimation} == {
        "static_text",
        "scrolling_text",
        "rainbow_wave",
        "fire",
        "plasma",
        "matrix_rain",
        "custom_pattern",
        "rose_color_wave",
        "custom_animation",
        "sad",
        "love",
        "happy",
        "angry",
        "manual_paint",
        "battery",
    }


def test_a_refused_display_command_surfaces_the_robots_reason(
    robot, transport
) -> None:
    # "no LED matrix on this series" comes back as ok:False inside a NORMAL
    # ack, not a protocol error, so nothing raises and the bare False the
    # caller sees would throw the only useful sentence away.
    transport.script_ack(
        protocol.CMD_DISPLAY_TEXT,
        {"ok": False, "error": "no LED matrix on this series"},
    )
    with pytest.warns(UserWarning, match="no LED matrix on this series"):
        assert robot.head.set_display_text("hi") is False


def test_a_successful_display_command_does_not_warn(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_TEXT, {"ok": True})
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert robot.head.set_display_text("hi") is True
