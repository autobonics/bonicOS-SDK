"""Speech — one method, the robot decides where the audio comes out (API.md §7).

Lite models (always have a tablet): the Flutter app serves the WS and does
Android TTS directly — fully live. Pro models: 🔌 stub until the ESP-relay
(with tablet) / on-device TTS (a2-pro, no tablet) path lands — the call
still runs and blocks until accepted, it just won't be audible yet.
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        ok = robot.speak("Hello, I am running the bonicos SDK examples.")
        print(f"speak(...) -> {ok}")

        ok = robot.speak("This call specifies a voice.", voice="en-US-default")
        print(f"speak(..., voice=...) -> {ok}")


if __name__ == "__main__":
    main()
