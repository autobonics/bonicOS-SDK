"""Built-in detectors — no training needed.

Each runs a model bundled in the robot image (``$BONICOS_MODELS_DIR``,
described by ``manifest.json``) with the runtime chosen for it on an A Pro's
RPi 4 (AITrainingAndInferenceOverview.md §14.0): YOLOX-nano and YuNet on
``cv2.dnn``, ArUco on ``cv2.aruco``, gestures on MediaPipe. Models load on
first use and are kept for the life of the program.

Speeds under run_code's CPU quota on an A Pro, per frame: objects ~205 ms at
the default 320 input (``input_size=256`` is faster, and misses more small
objects), faces ~36 ms, markers ~8 ms, gestures ~95 ms. Write loops that call
several of these with that in mind.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Tuple

from ..exceptions import AIUnavailable
from . import _env
from ._types import Detection, Face, Gesture, Marker, Point, as_bgr, cv2_module

_cache: Dict[Any, Any] = {}

_OBJECT_SIZES = (256, 320, 416)
_FACE_WIDTH = 320
_MARKER_DICTIONARIES = {
    "4x4_50": "DICT_4X4_50",
    "4x4_100": "DICT_4X4_100",
    "5x5_50": "DICT_5X5_50",
    "6x6_50": "DICT_6X6_50",
    "apriltag_36h11": "DICT_APRILTAG_36h11",
}
_FACE_LANDMARKS = ("right_eye", "left_eye", "nose", "mouth_right", "mouth_left")


def _builtin(name: str) -> Tuple[str, Dict[str, Any]]:
    """``(path to the model file, its manifest entry)``."""
    root, manifest = _env.manifest()
    entry = manifest["builtin"].get(name)
    if entry is None:
        raise AIUnavailable(
            f"this robot has no built-in {name} model — its software may need updating."
        )
    return str(root / entry["file"]), dict(entry, _root=str(root))


# ── objects ──────────────────────────────────────────────────────────────


def detect_objects(
    frame: Any, min_confidence: float = 0.4, input_size: int | None = None
) -> List[Detection]:
    """Everyday objects (the 80 COCO classes: person, cup, bottle, chair, …),
    most confident first.

    ``input_size`` trades accuracy for speed: 256, 320 (default) or 416."""
    path, entry = _builtin("objects")  # first: says "use a robot" before any import
    import numpy as np

    cv2 = cv2_module()
    img = as_bgr(frame)
    size = input_size or int(entry["inputSize"])
    if size not in _OBJECT_SIZES:
        raise ValueError(f"input_size must be one of {_OBJECT_SIZES}, got {size}")

    key = ("objects", path)
    if key not in _cache:
        with open(f"{entry['_root']}/{entry['labels']}") as f:
            names = [line.strip() for line in f if line.strip()]
        _cache[key] = (cv2.dnn.readNetFromONNX(path), names)
    net, names = _cache[key]

    # YOLOX's own preprocessing: letterbox into the top-left of a square padded
    # with 114, raw 0-255 BGR, NCHW. No normalisation.
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    padded = np.full((size, size, 3), 114, np.uint8)
    resized = cv2.resize(img, (int(w * r), int(h * r)), interpolation=cv2.INTER_LINEAR)
    padded[: resized.shape[0], : resized.shape[1]] = resized
    net.setInput(padded.transpose(2, 0, 1)[None].astype(np.float32))
    out = net.forward()[0]

    grid, stride = _yolox_grid(size)
    xy = (out[:, :2] + grid) * stride
    wh = np.exp(out[:, 2:4]) * stride
    scores = out[:, 4:5] * out[:, 5:]
    cls = scores.argmax(1)
    conf = scores[np.arange(len(cls)), cls]
    keep = conf >= min_confidence
    if not keep.any():
        return []
    boxes = np.concatenate([xy[keep] - wh[keep] / 2, wh[keep]], 1) / r
    cls, conf = cls[keep], conf[keep]
    idx = np.array(
        cv2.dnn.NMSBoxesBatched(
            boxes.tolist(), conf.tolist(), cls.tolist(), min_confidence, 0.45
        )
    ).flatten()

    found = []
    for i in idx:
        x0, y0, bw, bh = boxes[i]
        x0, y0 = max(0, int(x0)), max(0, int(y0))
        x1, y1 = min(w, int(boxes[i][0] + bw)), min(h, int(boxes[i][1] + bh))
        found.append(
            Detection(
                label=names[cls[i]],
                confidence=round(float(conf[i]), 3),
                x=x0,
                y=y0,
                width=max(0, x1 - x0),
                height=max(0, y1 - y0),
            )
        )
    return sorted(found, key=lambda d: -d.confidence)


def _yolox_grid(size: int) -> Tuple[Any, Any]:
    import numpy as np

    key = ("yolox_grid", size)
    if key not in _cache:
        grids, strides = [], []
        for s in (8, 16, 32):
            n = size // s
            xv, yv = np.meshgrid(np.arange(n), np.arange(n))
            grids.append(np.stack((xv, yv), 2).reshape(-1, 2))
            strides.append(np.full((n * n, 1), s))
        _cache[key] = (np.concatenate(grids), np.concatenate(strides))
    grid, stride = _cache[key]
    return grid, stride


# ── faces ────────────────────────────────────────────────────────────────


def detect_faces(frame: Any, min_confidence: float = 0.6) -> List[Face]:
    """Faces, most confident first, each with five landmarks (eyes, nose,
    mouth corners). Works best on faces facing the camera."""
    path, _ = _builtin("faces")  # first: says "use a robot" before any import
    cv2 = cv2_module()
    img = as_bgr(frame)
    key = ("faces", path)
    if key not in _cache:
        _cache[key] = cv2.FaceDetectorYN.create(path, "", (_FACE_WIDTH, _FACE_WIDTH))
    det = _cache[key]

    # Detect on a copy scaled to 320 wide — 25 ms rather than 121 ms at 640x480
    # on an A Pro, and it still found a small, distant face in testing.
    h, w = img.shape[:2]
    s = min(1.0, _FACE_WIDTH / w)
    small = cv2.resize(img, (round(w * s), round(h * s))) if s < 1.0 else img
    det.setInputSize((small.shape[1], small.shape[0]))
    det.setScoreThreshold(float(min_confidence))
    _, rows = det.detect(small)
    if rows is None:
        return []

    faces = []
    for row in rows:
        v = [int(round(float(n) / s)) for n in row[:14]]
        faces.append(
            Face(
                x=max(0, v[0]),
                y=max(0, v[1]),
                width=v[2],
                height=v[3],
                confidence=round(float(row[14]), 3),
                landmarks={
                    k: (v[4 + 2 * i], v[5 + 2 * i])
                    for i, k in enumerate(_FACE_LANDMARKS)
                },
            )
        )
    return sorted(faces, key=lambda f: -f.confidence)


# ── markers ──────────────────────────────────────────────────────────────


def detect_markers(frame: Any, dictionary: str = "4x4_50") -> List[Marker]:
    """ArUco markers, as printed from ``dictionary`` (default ``"4x4_50"``;
    also ``"4x4_100"``, ``"5x5_50"``, ``"6x6_50"``, ``"apriltag_36h11"``).
    Needs no model file, so it also works off-robot with OpenCV installed —
    but not in the browser simulator."""
    _env.check_runtime()  # first: says "use a robot" before any import
    cv2 = cv2_module()
    img = as_bgr(frame)
    if dictionary not in _MARKER_DICTIONARIES:
        raise ValueError(
            f"dictionary must be one of {sorted(_MARKER_DICTIONARIES)}, "
            f"got {dictionary!r}"
        )
    key = ("markers", dictionary)
    if key not in _cache:
        d = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, _MARKER_DICTIONARIES[dictionary])
        )
        _cache[key] = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = _cache[key].detectMarkers(img)
    if ids is None:
        return []
    markers = []
    for c, marker_id in zip(corners, ids.flatten()):
        pts = tuple((int(round(p[0])), int(round(p[1]))) for p in c.reshape(4, 2))
        markers.append(Marker(id=int(marker_id), corners=pts))  # type: ignore[arg-type]
    return sorted(markers, key=lambda m: m.id)


# ── gestures ─────────────────────────────────────────────────────────────


def detect_gestures(frame: Any, max_hands: int = 1) -> List[Gesture]:
    """Hands and the gesture each is making.

    ``max_hands=1`` (the default) is more than twice as fast as 2: with one
    hand in view MediaPipe tracks it from frame to frame instead of searching
    the whole image each time (~95 ms vs ~220 ms on an A Pro).

    Because it tracks, a hand that suddenly replaces a different one — a cut,
    not movement — is found one frame later. A hand entering an empty view is
    found at once, and a live camera rarely cuts."""
    path, _ = _builtin("gestures")  # first: says "use a robot" before any import
    import numpy as np

    cv2 = cv2_module()
    img = as_bgr(frame)
    try:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision as mp_vision
    except ImportError:
        raise AIUnavailable(
            "gesture detection needs MediaPipe, which is installed on the robot."
        ) from None

    key = ("gestures", path, max_hands)
    if key not in _cache:
        options = mp_vision.GestureRecognizerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=path),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=int(max_hands),
        )
        _cache[key] = [mp_vision.GestureRecognizer.create_from_options(options), -1]
    recognizer, last_ts = _cache[key]
    # VIDEO mode needs strictly increasing timestamps; two calls within the
    # same millisecond would otherwise raise.
    ts = max(int(time.monotonic() * 1000), last_ts + 1)
    _cache[key][1] = ts

    rgb = np.ascontiguousarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    result = recognizer.recognize_for_video(
        mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts
    )

    h, w = img.shape[:2]
    hands = []
    for i, points in enumerate(result.hand_landmarks):
        top = result.gestures[i][0] if result.gestures and result.gestures[i] else None
        pts: Tuple[Point, ...] = tuple(
            (min(w - 1, max(0, int(p.x * w))), min(h - 1, max(0, int(p.y * h))))
            for p in points
        )
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        hands.append(
            Gesture(
                name=top.category_name if top else "None",
                confidence=round(float(top.score), 3) if top else 0.0,
                hand=_persons_hand(result.handedness[i][0].category_name),
                landmarks=pts,
                x=min(xs),
                y=min(ys),
                width=max(xs) - min(xs),
                height=max(ys) - min(ys),
            )
        )
    return hands


def _persons_hand(label: str) -> str:
    """MediaPipe labels handedness assuming a mirrored, selfie-style image, and
    says to swap it otherwise. A robot's camera is not mirrored, so swap —
    that makes ``hand`` the person's own right or left."""
    return {"Left": "Right", "Right": "Left"}.get(label, label)
