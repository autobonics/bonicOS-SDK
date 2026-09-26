"""Speech — say things, and choose how (API.md §7).

The robot decides where the words come out: the BonicOS app on a robot with
BonicOS, otherwise the robot's own on-device voice.
"""

from bonicos import BonicBot, CommandError

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    with BonicBot(HOST) as robot:
        robot.speak("Hello, I am running the bonicos SDK examples.")
        robot.speak("This sentence is a little slower.", rate=0.8)

        # Cloud voices speak many more languages and are paid for from the
        # robot's credits. A robot without BonicOS refuses them, and says so.
        try:
            robot.speak("നമസ്കാരം", language="ml-IN", engine="cloud")
            robot.speak("This is a chosen cloud voice.", "Puck", engine="cloud")
        except CommandError as e:
            print(f"cloud speech unavailable: {e}")


if __name__ == "__main__":
    main()
