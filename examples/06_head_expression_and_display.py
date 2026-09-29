"""Head expression, look, and LED-matrix display (API.md §6).

**Expressions are live on A and S series**; the LED-matrix `display_*` calls
are A series only. Two things worth knowing before you run this:

  * **The base stack must be up.** The robot's hardware plugin owns the serial
    link to the face, so with the stack down there is no path to it at all —
    every call below raises `CommandError` with the robot's reason.
  * **A robot refuses what its face cannot show.** The S display shows preset
    expressions only, so `display_*` raises there; a robot with no face
    display (M series) refuses everything here but `look()`.

`look()` moves the neck; everything else only changes the face.
"""

from bonicos import BonicBot, CommandError, DisplayAnimation, HeadMode

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        # Every HeadMode is a real face, on the A matrix and the S display.
        for mode in HeadMode:
            robot.set_expression(mode)
            print(f"set_expression({mode.value})")

        # Plain strings work too — HeadMode is just for discoverability.
        robot.set_expression("happy")

        # Neck aim, in DEGREES. `tilt` is neck pitch, which an A2 does not
        # fit: pan + tilt moves the pan and warns; tilt alone raises.
        robot.look(pan=20, tilt=-10, duration=1.5)
        robot.look(pan=0, tilt=0)

        # The LED matrix (A series). On the S display these raise, so the
        # whole block is one try — the robot's reason says why.
        try:
            # Text and colour. ASCII only — the panel's font has nothing else.
            robot.set_display_color(r=0, g=200, b=255)
            robot.set_display_text("Hello from bonicos!")

            # Brightness is a raw 0-255 byte, NOT a 0..1 fraction: 0.8 is off.
            robot.set_display_brightness(200)

            # Animations by name (DisplayAnimation), or a raw firmware index
            # for anything the enum hasn't named yet.
            robot.set_display_animation(DisplayAnimation.RAINBOW_WAVE)
            robot.pause_display()
            robot.play_display()

            robot.clear_display()
            print("Display sequence sent.")
        except CommandError as e:
            print(f"no LED matrix on this robot: {e}")


if __name__ == "__main__":
    main()
