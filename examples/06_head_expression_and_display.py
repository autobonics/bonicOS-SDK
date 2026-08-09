"""Head expression, look, and LED-matrix display (API.md §6).

**All of this is 🔌 stub in v1** — no ROS path exists yet for the head/LED
hardware, so the server acks these as safe no-ops (log + ack, PROTOCOL §5.5).
The calls below run for real over the wire and exercise the full API/protocol
contract; the robot just won't visibly react until the server side lands.
Nothing here can move the base or arms.
"""

from bonicos import BonicBot, HeadMode

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        robot.wait_for_data()

        for mode in (HeadMode.HAPPY, HeadMode.SURPRISED, HeadMode.NORMAL):
            ok = robot.set_expression(mode)
            print(f"set_expression({mode.value}) -> {ok}")

        # Plain strings work too — HeadMode is just for discoverability.
        robot.set_expression("confused")

        ok = robot.look(pan=20, tilt=-10, speed=30)
        print(f"look(pan=20, tilt=-10) -> {ok}")

        robot.set_display_text("Hello from bonicos!")
        robot.set_display_color(r=0, g=200, b=255)
        robot.set_display_brightness(0.8)
        robot.set_display_animation("pulse")
        robot.play_display()
        robot.pause_display()
        robot.clear_display()
        print("Display sequence sent (no-op on hardware in v1).")


if __name__ == "__main__":
    main()
