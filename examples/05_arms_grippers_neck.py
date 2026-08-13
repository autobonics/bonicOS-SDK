"""Arms, grippers, and neck — servo control (API.md §5).

Angles are **degrees** at this API boundary (converted to radians on the
wire). `wait=True` (the default on set_servos/move_*_arm) blocks until the
joints actually converge, not just until the server acks the command.
Partial calls are safe — you never need to name every joint in a
controller group; the SDK holds the rest at their current position.

Known sim caveat (not an SDK bug): on the current M1 Gazebo sim,
leftElbow/rightElbow specifically don't respond to position commands —
every other joint does.

This moves the robot. Set HOST below before running.
"""

from bonicos import BonicBot, ServoID

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    input(
        "This will physically move the robot's arms. Press Enter to continue "
        "(Ctrl+C aborts)..."
    )

    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        # Multiple joints in one command, by name.
        ok = robot.set_servos({"leftShoulderPitch": 45, "neckYaw": 20}, duration=1.5)
        print(f"set_servos({{leftShoulderPitch, neckYaw}}) -> {ok}")

        # Arm shorthands (shoulder, elbow) — elbow kept <= 0 per the
        # registry's range (a positive elbow target won't move on M1).
        ok = robot.move_left_arm(shoulder=60, elbow=-30)
        print(f"move_left_arm(60, -30) -> {ok}")
        ok = robot.move_right_arm(shoulder=60, elbow=-30)
        print(f"move_right_arm(60, -30) -> {ok}")

        # Grippers.
        robot.open_grippers()
        robot.close_grippers()
        robot.set_grippers(left=50, right=50)

        # Neck / look.
        robot.look_left()
        robot.look_right()
        robot.look_center()
        robot.set_neck(yaw=15)

        # Longer duration + an explicit convergence timeout.
        ok = robot.move_left_arm(shoulder=30, elbow=-10, duration=2.0, timeout=8.0)
        print(f"move_left_arm(duration=2.0, timeout=8.0) -> {ok}")

        # Read back current joint positions (registry camelCase keys, the
        # same names you command with).
        angles = robot.get_servo_angles()
        left_shoulder = angles.get(ServoID.LEFT_SHOULDER_PITCH.value)
        print("leftShoulderPitch is now:", left_shoulder)

        # Fire-and-forget: a second command to the same group preempts the
        # first mid-trajectory rather than queuing — safe to chain quickly.
        robot.move_right_arm(shoulder=0, elbow=0, wait=False)
        robot.move_right_arm(shoulder=90, elbow=-45, wait=False)

        robot.reset_servos()
        print("All 18 registry joints reset to neutral.")


if __name__ == "__main__":
    main()
