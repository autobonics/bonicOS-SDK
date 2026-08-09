"""Connect to a robot, inspect the handshake, then disconnect cleanly.

Covers: BonicBot(), is_connected(), features/robot_id/series, close(), and
the context-manager form that guarantees a stop on exit (API.md §1).
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    robot = BonicBot(HOST, robot_id=ROBOT_ID)
    print(f"Connected: robot_id={robot.robot_id!r} series={robot.series!r}")
    print("is_connected:", robot.is_connected())
    for feature, enabled in sorted(robot.features.items()):
        print(f"  feature {feature}: {enabled}")

    robot.close()
    print("is_connected after close():", robot.is_connected())
    robot.close()  # idempotent — safe to call again

    # Preferred form for real programs: guarantees stop() + socket close even
    # on an exception, matching the platform's "stop in a finally" rule.
    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        print("Reconnected via context manager:", robot.robot_id)
        # ... do work here ...
    print("Context manager exited — motors stopped, socket closed.")


if __name__ == "__main__":
    main()
