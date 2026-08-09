"""Sensors & telemetry — non-blocking cached reads (API.md §8).

Telemetry streams continuously and is cached; get_*() calls never block —
they just return the latest known value. `wait_for_update()` is how you
pace a loop to the real sensor rate instead of busy-spinning. `subscribe()`
narrows the stream (handy to avoid a client-side backlog from a full-rate
firehose like imu/joint_states).
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        got_data = robot.wait_for_data(timeout=5.0)
        print(f"wait_for_data() -> {got_data}")

        print("position:", robot.get_position())
        print("x, y, heading:", robot.get_x(), robot.get_y(), robot.get_heading())
        print("battery: %.1f%%" % robot.get_battery())
        print("imu:", robot.get_imu())
        print("distance_traveled since connect:", robot.get_distance_traveled())

        # Narrow the telemetry stream to just what this program cares about.
        robot.subscribe(["pose", "battery"])

        # Recommended loop pattern (bonic-architecture.md §5): never spins,
        # self-paces to the sensor rate, identical in the browser. Bounded to
        # a handful of iterations here so the example terminates on its own.
        print("Polling telemetry for up to 10 updates...")
        for _ in range(10):
            if not robot.wait_for_update(timeout=2.0):
                print("  no update within timeout — stopping early")
                break
            print(f"  battery={robot.get_battery():.1f}%  pos={robot.get_position()}")


if __name__ == "__main__":
    main()
