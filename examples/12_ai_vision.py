"""Computer vision with `bonicos.ai` (API.md §9.1).

Unlike the other examples, this one must run ON THE ROBOT — from the Code tab
with the robot as the run target. The models are part of the robot's software,
so on a laptop (or in the simulator) every `ai` call raises `AIUnavailable`
saying exactly that.

It watches the camera for ten seconds and reports what it sees. To try a model
you trained, set MY_MODEL to its name in the Train tab.
"""

import time

from bonicos import BonicBot, ai

MY_MODEL = None  # e.g. "cup-detector"


def main() -> None:
    with BonicBot() as robot:
        mine = ai.load(MY_MODEL) if MY_MODEL else None
        end = time.time() + 10
        while time.time() < end:
            frame = robot.get_camera_frame()
            if frame is None:  # the camera has not delivered its first frame yet
                continue

            if mine:
                label, confidence = mine.predict(frame)[0]
                print(f"{MY_MODEL}: {label} ({confidence:.0%})")

            for thing in ai.detect_objects(frame):
                print(f"{thing.label} ({thing.confidence:.0%}) at {thing.center}")

            for face in ai.detect_faces(frame):
                print(f"face at {face.center}")

            for hand in ai.detect_gestures(frame):
                if hand.name == "Thumb_Up":
                    robot.speak("Thumbs up!")
                print(f"{hand.hand} hand: {hand.name}")

            for marker in ai.detect_markers(frame):
                print(f"marker {marker.id} at {marker.center}")


if __name__ == "__main__":
    main()
