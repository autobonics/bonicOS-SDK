"""Exceptions the SDK raises, and how to handle them (API.md §11).

`FeatureUnavailable` and `RobotDisconnected` are real Python exceptions
that surface inside your code (not error return codes) so student programs
can `try`/`except` them like anything else. All bonicos exceptions share a
`RobotError` base, so a single `except RobotError` catches all of them.
"""

from bonicos import (
    BonicBot,
    CommandError,
    FeatureUnavailable,
    RobotDisconnected,
    RobotError,
)
from bonicos import ConnectionError as BonicConnectionError

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def demo_connection_error() -> None:
    print("\n-- ConnectionError: bad host --")
    try:
        BonicBot("192.0.2.1", robot_id="NOPE", timeout=2.0)  # TEST-NET-1, never routes
    except BonicConnectionError as exc:
        print(f"Caught ConnectionError as expected: {exc}")


def demo_feature_unavailable(robot: BonicBot) -> None:
    print("\n-- FeatureUnavailable: a gated feature on this series --")
    for feature, enabled in robot.features.items():
        if not enabled:
            try:
                # Any nav call is gated the same way; go_to is representative.
                robot.go_to(0.0, 0.0)
            except FeatureUnavailable as exc:
                print(f"Caught FeatureUnavailable for {feature!r}: {exc}")
            return
    print("Every advertised feature is enabled here — nothing to trigger.")


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

    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        robot.wait_for_data()
        demo_feature_unavailable(robot)
        demo_general_pattern(robot)


if __name__ == "__main__":
    main()
