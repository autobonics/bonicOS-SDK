"""``bonicos.ai`` — computer vision on the robot (API.md §9.1).

    from bonicos import BonicBot, ai

    with BonicBot() as robot:
        cups = ai.load("cup-detector")           # a model you trained in the Train tab
        while True:
            frame = robot.get_camera_frame()
            if frame is None:
                continue
            label, confidence = cups.predict(frame)[0]
            for thing in ai.detect_objects(frame):   # built in, no training
                print(thing.label, thing.confidence, thing.center)

Every function takes a camera frame (a BGR image, as ``get_camera_frame``
returns) and is independent of any robot connection. The models run **on the
robot**: they are bundled in its image, and trained models are sent with the
program that uses them. Importing this module is always safe — in the browser
simulator, or anywhere the models are not installed, each call raises
:class:`AIUnavailable` with a sentence saying why.
"""

from __future__ import annotations

from ..exceptions import AIUnavailable, InvalidModel, ModelNotFound
from ._builtin import detect_faces, detect_gestures, detect_markers, detect_objects
from ._classifier import Model, list_models, load
from ._types import Detection, Face, Gesture, Marker, Prediction

__all__ = [
    "load",
    "list_models",
    "detect_objects",
    "detect_faces",
    "detect_markers",
    "detect_gestures",
    "Model",
    "Prediction",
    "Detection",
    "Face",
    "Marker",
    "Gesture",
    "AIUnavailable",
    "ModelNotFound",
    "InvalidModel",
]
