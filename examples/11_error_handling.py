"""Exceptions the SDK raises, and how to handle them (API.md §11).

`CommandError` and `RobotDisconnected` are real Python exceptions that
surface inside your code (not error return codes) so student programs can
`try`/`except` them like anything else. All bonicos exceptions share a
`RobotError` base, so a single `except RobotError` catches all of them.

Note `CommandError` also covers "this robot can't do that": capability is
never advertised or checked client-side (PROTOCOL.md §3.1), so asking a Lite
robot to navigate is an ordinary command error carrying the robot's own
explanation.
"""

from bonicos import (
    BonicBot,
    CommandError,
    RobotDisconnected,
    RobotError,
)
from bonicos import ConnectionError as BonicConnectionError

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def demo_connection_error() -> None:
    print("\n-- ConnectionError: bad host --")
    try:
        BonicBot("192.0.2.1", timeout=2.0)  # TEST-NET-1, never routes
    except BonicConnectionError as exc:
        print(f"Caught ConnectionError as expected: {exc}")


def demo_unsupported_command(robot: BonicBot) -> None:
    print("\n-- CommandError: something this robot cannot do --")
    try:
        # On a Lite robot (no lidar, no on-board computer) this fails; on a
        # Pro robot it succeeds. Either way the SDK made no prediction.
        robot.go_to(0.0, 0.0, wait=False)
        print("This robot has navigation — no error to show.")
    except CommandError as exc:
        print(f"Caught CommandError: {exc}")


def demo_general_pattern(robot: BonicBot) -> None:
    print("\n-- General pattern: one except clause covers everything --")
    try:
        robot.speak("Testing broad error handling.")
        # CommandError: raised if the server replies with an `error` frame
        # (e.g. a malformed/rejected command) — same shape as the others.
        # RobotDisconnected: raised if the link drops mid-call, e.g.
        #   robot.close(); robot.drive_distance(1.0)  # -> RobotDisconnected
    except (CommandError, RobotDisconnected) as exc:
        print(f"Command-level or disconnect error: {exc}")
    except RobotError as exc:
        # Catch-all for any bonicos exception you didn't specifically expect.
        print(f"Some other bonicos error: {exc}")
    else:
        print("speak() succeeded, no errors.")


def main() -> None:
    demo_connection_error()

    with BonicBot(HOST) as robot:
        robot.wait_for_data()
        demo_unsupported_command(robot)
        demo_general_pattern(robot)


if __name__ == "__main__":
    main()
