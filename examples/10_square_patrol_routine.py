"""A small combined routine: patrol a square, watch the battery, speak status.

This is the API.md §12 "square patrol" worked example, expanded with the
recommended battery-check loop pattern from §8. It's the shape a real
student/developer program takes: connect, do bounded work, react to
telemetry, clean up — everything else in this examples/ directory is one
capability at a time; this one ties several together.

This moves the robot. Set HOST below before running.
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
LOW_BATTERY_PCT = 15.0


def main() -> None:
    input(
        "This will drive the robot in a ~1m square. Press Enter to continue "
        "(Ctrl+C aborts)..."
    )

    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        battery = robot.get_battery()
        print(f"Starting battery: {battery:.1f}%")
        if battery < LOW_BATTERY_PCT:
            robot.speak("Battery too low to patrol.")
            return

        for leg in range(4):
            print(f"Leg {leg + 1}/4...")
            robot.drive_distance(1.0)
            robot.rotate_angle(90)

            if robot.get_battery() < LOW_BATTERY_PCT:
                robot.speak("Low battery, aborting patrol early.")
                break
        else:
            robot.speak("Patrol complete.")

        print(f"Ending battery: {robot.get_battery():.1f}%")
        print(f"Distance traveled: {robot.get_distance_traveled():.2f}m")


if __name__ == "__main__":
    main()
