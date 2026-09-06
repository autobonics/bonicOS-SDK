"""Head expression, look, and LED-matrix display (API.md §6).

**Live on A series.** robot_app packs the `CMD_MATRIX_ACTION` body and
publishes it to the ros2_control plugin, which forwards it to the ESP over
USB CDC. Two consequences worth knowing before you run this:

  * **The base stack must be up.** The plugin owns `/dev/esp` and only one
    process may hold it, so with the stack down there is no path to the panel
    at all — every call below returns False with the robot's reason.
  * **A series without an LED matrix says so.** M1's head is not wired this
    way and nothing subscribes the topic, so these refuse rather than
    silently succeeding.

`look()` moves the neck; everything else only lights pixels.
"""

import warnings

from bonicos import BonicBot, DisplayAnimation, HeadMode

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        for mode in (HeadMode.HAPPY, HeadMode.SAD, HeadMode.NORMAL):
            ok = robot.set_expression(mode)
            print(f"set_expression({mode.value}) -> {ok}")

        # Two of the six have no face in firmware and arrive as documented
        # approximations — surprised is a heart, confused a colour effect.
        # The call succeeds and warns; catching it is how you'd check before
        # building a lesson around either.
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            robot.set_expression(HeadMode.SURPRISED)
            for w in caught:
                print("note:", w.message)

        # Plain strings work too — HeadMode is just for discoverability.
        robot.set_expression("angry")

        # Neck aim, in DEGREES. `tilt` is neck pitch, which an A2 does not
        # fit, so tilt alone on one returns False — nothing moved.
        ok = robot.look(pan=20, tilt=-10, duration=1.5)
        print(f"look(pan=20, tilt=-10) -> {ok}")
        robot.look(pan=0, tilt=0)

        # Text and colour. ASCII only — the panel's font has nothing else.
        robot.set_display_color(r=0, g=200, b=255)
        robot.set_display_text("Hello from bonicos!")

        # Brightness is a raw 0-255 byte, NOT a 0..1 fraction: 0.8 is off.
        robot.set_display_brightness(200)

        # Animations by name (DisplayAnimation), or a raw firmware index for
        # anything the enum hasn't named yet.
        robot.set_display_animation(DisplayAnimation.RAINBOW_WAVE)
        robot.pause_display()
        robot.play_display()

        robot.clear_display()
        print("Display sequence sent.")


if __name__ == "__main__":
    main()
