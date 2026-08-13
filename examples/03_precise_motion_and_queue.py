"""Closed-loop precise motion + the command queue (API.md §3).

`drive_distance`/`rotate_angle`/`drive_and_rotate`/`draw_square` are
client-side control loops over `drive()` + odometry — blocking, with a
timeout, backstopped by the on-robot deadman if the connection stalls.
The queue methods let you build a routine first and run it as one unit.

This moves the robot. Set HOST below before running.
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    input(
        "This will physically move the robot. Press Enter to continue (Ctrl+C aborts)"
    )

    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        ok = robot.drive_distance(0.5, speed=0.3, timeout=15.0)
        print(f"drive_distance(0.5m) -> {ok}")

        ok = robot.rotate_angle(90, speed=45.0, timeout=15.0)
        print(f"rotate_angle(90deg) -> {ok}")

        ok = robot.drive_and_rotate(0.3, -90, speed=0.3, turn_speed=45.0)
        print(f"drive_and_rotate(0.3m, -90deg) -> {ok}")

        ok = robot.draw_square(0.4, speed=0.3, turn_speed=45.0)
        print(f"draw_square(0.4m side) -> {ok}")

        # Build a routine, then run it in one call.
        robot.enqueue([("drive", 0.3), ("rotate", 90), ("drive", 0.3), ("rotate", -90)])
        ok = robot.run_queue(block=True)
        print(f"run_queue() -> {ok}")

        # clear_queue() flushes anything pending and stops the robot — handy
        # as a panic button if a routine needs to be aborted mid-run.
        robot.enqueue([("drive", 5.0)])
        robot.clear_queue()
        print("Queue cleared before running — nothing executed.")


if __name__ == "__main__":
    main()
