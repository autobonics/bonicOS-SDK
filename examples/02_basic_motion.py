"""Basic base motion: the timed helpers plus raw drive() (API.md §2).

`duration=None` starts the motion and returns immediately; a number blocks
for that long then stops. Real bases have a minimum speed threshold below
which they won't move at all — keep speed >= 0.2 m/s.

This moves the robot. Set HOST below before running.
"""

import time

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    input(
        "This will physically move the robot. Press Enter to continue (Ctrl+C aborts)"
    )

    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        robot.wait_for_data()

        print("Moving forward for 2s...")
        robot.move_forward(speed=0.3, duration=2)

        print("Moving backward for 2s...")
        robot.move_backward(speed=0.3, duration=2)

        print("Turning left for 1s...")
        robot.turn_left(speed=0.5, duration=1)

        print("Turning right for 1s...")
        robot.turn_right(speed=0.5, duration=1)

        # Fire-and-forget form: drive() keeps streaming the same command to
        # satisfy the on-robot deadman until you call stop() or drive again.
        print("Raw drive() for 1s, then stop()...")
        robot.drive(linear_x=0.2, angular_z=0.3)
        time.sleep(1)
        print("is_moving() while driving:", robot.is_moving())
        robot.stop()
        time.sleep(0.5)  # sim/hardware deceleration ramp before the read settles
        print("is_moving() after stop():", robot.is_moving())


if __name__ == "__main__":
    main()
