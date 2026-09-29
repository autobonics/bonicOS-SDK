"""Head expression & face display (API.md §6).

These pin the unit conversion on ``look``, and telling the caller when the
robot did not actually do what was asked.
"""

from __future__ import annotations

import math
import warnings

import pytest

from bonicos import protocol
from bonicos.enums import DisplayAnimation, HeadMode
from bonicos.exceptions import CommandError


def test_set_expression_sends_the_emotion(robot, transport) -> None:
    transport.script_ack(protocol.CMD_EMOTION, {"ok": True, "emotion": "happy"})
    assert robot.head.set_expression(HeadMode.HAPPY) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_EMOTION
    assert sent["emotion"] == "happy"


def test_set_expression_accepts_a_bare_string(robot, transport) -> None:
    transport.script_ack(protocol.CMD_EMOTION, {"ok": True})
    assert robot.head.set_expression("angry") is True
    assert transport.sent[-1]["emotion"] == "angry"


def test_an_unknown_expression_raises_with_the_known_names(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_EMOTION,
        {"ok": False, "error": "unknown emotion: 'smug'", "known": ["happy", "sad"]},
    )
    with pytest.raises(CommandError, match="unknown emotion") as err:
        robot.head.set_expression("smug")
    assert err.value.result["known"] == ["happy", "sad"]


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


def test_look_raises_when_the_robot_has_none_of_the_axes_asked_for(
    robot, transport
) -> None:
    # tilt alone on an A2: it has no neck pitch, so nothing moved.
    transport.script_ack(
        protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": ["neck_pitch_joint"]}
    )
    with pytest.raises(CommandError, match="no tilt joint"):
        robot.head.look(tilt=20.0)


def test_look_is_true_when_some_axis_moved_and_warns_about_the_rest(
    robot, transport
) -> None:
    transport.script_ack(
        protocol.CMD_HEAD_LOOK, {"ok": True, "unsupported": ["neck_pitch_joint"]}
    )
    with pytest.warns(UserWarning, match="no tilt joint"):
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
    assert transport.sent[-1]["mode"] == "scroll"

    assert robot.head.set_display_text("ok", mode="static") is True
    assert transport.sent[-1]["mode"] == "static"

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
        "surprised",
        "confused",
    }


def test_a_refused_display_command_surfaces_the_robots_reason(
    robot, transport
) -> None:
    # An ok:False ack raises with the robot's reason.
    transport.script_ack(
        protocol.CMD_DISPLAY_TEXT,
        {"ok": False, "error": "no LED matrix on this series"},
    )
    with pytest.raises(CommandError, match="no LED matrix on this series"):
        robot.head.set_display_text("hi")


def test_a_successful_display_command_does_not_warn(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_TEXT, {"ok": True})
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert robot.head.set_display_text("hi") is True


def test_love_is_a_head_mode(robot, transport) -> None:
    transport.script_ack(
        protocol.CMD_EMOTION, {"ok": True, "emotion": "love", "emotion_id": 6}
    )
    assert robot.head.set_expression(HeadMode.LOVE) is True
    assert transport.sent[-1]["emotion"] == "love"


def test_an_expressions_only_display_refusal_raises_with_the_reason(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_TEXT, {
        "ok": False, "error": "this robot's face shows preset expressions only"})
    with pytest.raises(CommandError, match="preset expressions only"):
        robot.head.set_display_text("hi")


def test_set_display_pixel_sends_the_position_and_colour(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_PIXEL, {"ok": True})
    assert robot.head.set_display_pixel(3, 4, 255, 0, 8) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_DISPLAY_PIXEL
    assert (sent["x"], sent["y"], sent["r"], sent["g"], sent["b"]) == (3, 4, 255, 0, 8)


def test_set_display_frame_flattens_rows_top_row_first(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_FRAME, {"ok": True})
    w, h = protocol.DISPLAY_WIDTH, protocol.DISPLAY_HEIGHT
    rows = [[(y, x, 0) for x in range(w)] for y in range(h)]
    assert robot.head.set_display_frame(rows) is True
    sent = transport.sent[-1]
    assert sent["type"] == protocol.CMD_DISPLAY_FRAME
    assert len(sent["pixels"]) == w * h
    assert sent["pixels"][0] == [0, 0, 0]
    assert sent["pixels"][w + 2] == [1, 2, 0]


def test_set_display_frame_takes_a_flat_list_as_is(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_FRAME, {"ok": True})
    flat = [[255, 0, 0]] * (protocol.DISPLAY_WIDTH * protocol.DISPLAY_HEIGHT)
    assert robot.head.set_display_frame(flat) is True
    assert transport.sent[-1]["pixels"] == flat


class _Array:
    """Stands in for a numpy array: only ``tolist()`` is used."""

    def __init__(self, data):
        self._data = data

    def tolist(self):
        return self._data


def test_a_wrong_shaped_frame_raises_without_sending(robot, transport) -> None:
    w, h = protocol.DISPLAY_WIDTH, protocol.DISPLAY_HEIGHT
    bad = [
        [[0, 0, 0]] * (w * h - 1),
        [[0, 0]] * (w * h),
        [[[0, 0, 0]] * w] * (h - 1),
        [[0, 0, "red"]] * (w * h),
        [[0, 0, float("nan")]] * (w * h),
        "red",
        None,
    ]
    for pixels in bad:
        with pytest.raises(CommandError, match="pixels must be"):
            robot.head.set_display_frame(pixels)
    assert transport.sent == []


def test_set_display_frame_takes_arrays_and_generators(robot, transport) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_FRAME, {"ok": True})
    w, h = protocol.DISPLAY_WIDTH, protocol.DISPLAY_HEIGHT
    assert robot.head.set_display_frame(_Array([[[9, 8, 7.5]] * w] * h)) is True
    assert transport.sent[-1]["pixels"][0] == [9, 8, 7]
    rows = ([(x, y, 0) for x in range(w)] for y in range(h))
    assert robot.head.set_display_frame(rows) is True
    assert transport.sent[-1]["pixels"][-1] == [w - 1, h - 1, 0]


def test_set_display_pixel_takes_whole_floats_and_array_scalars(
    robot, transport
) -> None:
    transport.script_ack(protocol.CMD_DISPLAY_PIXEL, {"ok": True})
    assert robot.head.set_display_pixel(6.0, _Array(2), 255, 0, 0) is True
    sent = transport.sent[-1]
    assert (sent["x"], sent["y"]) == (6, 2)
    assert isinstance(sent["x"], int)


def test_a_pixel_outside_the_drawing_area_raises_without_sending(
    robot, transport
) -> None:
    for x, y in ((12, 0), (0, 5), (-1, 0), (1.5, 0), (True, 0), (None, 0)):
        with pytest.raises(CommandError, match="x must be 0-11 and y 0-4"):
            robot.head.set_display_pixel(x, y, 0, 0, 0)
    with pytest.raises(CommandError, match="r, g and b"):
        robot.head.set_display_pixel(0, 0, "red", 0, 0)
    assert transport.sent == []


def test_an_unknown_text_mode_raises_without_sending(robot, transport) -> None:
    with pytest.raises(CommandError, match="unknown text mode") as err:
        robot.head.set_display_text("hi", mode="wave")
    assert err.value.result["known"] == ["scroll", "static"]
    assert transport.sent == []
    transport.script_ack(protocol.CMD_DISPLAY_TEXT, {"ok": True})
    assert robot.head.set_display_text("hi", mode=" Static ") is True
    assert transport.sent[-1]["mode"] == "static"
