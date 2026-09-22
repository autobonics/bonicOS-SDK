"""bonicos.ai — environment resolution, bonic-head-v1 loading/validation, and
the built-in detectors' plumbing.

Uses the real bundled models from a sibling bonicOS-robot-app checkout
(deploy/models) and skips when it is absent. Trained heads are generated here
in the exact bonic-head-v1 layout (AITrainingAndInferenceOverview.md §6.2).
Accuracy on real photographs is checked on robot hardware, not here.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import pytest

np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from bonicos import ai  # noqa: E402
from bonicos.ai import _builtin, _classifier, _env  # noqa: E402

MODELS = Path(__file__).resolve().parents[2] / "bonicOS-robot-app" / "deploy" / "models"
needs_models = pytest.mark.skipif(
    not (MODELS / "manifest.json").is_file(),
    reason="needs a bonicOS-robot-app checkout beside this repo",
)


@pytest.fixture(autouse=True)
def _clean(monkeypatch: pytest.MonkeyPatch) -> None:
    for var in (_env.MODELS_DIR_ENV, _env.CUSTOM_MODELS_ENV, _env.HEADS_DIR_ENV):
        monkeypatch.delenv(var, raising=False)
    _classifier._models.clear()
    _builtin._cache.clear()


def _manifest() -> Dict[str, Any]:
    return json.loads((MODELS / "manifest.json").read_text())


def _write_head(
    heads: Path,
    classes: List[str],
    *,
    hidden: int = 16,
    seed: int = 0,
    tweak: Any = None,
    base: str = "mobilenet_v2_128",
) -> str:
    """Write a bonic-head-v1 head the way the browser does; return its sha256."""
    bb = _manifest()["backbones"][base]
    rng = np.random.default_rng(seed)
    dims = [(bb["featureDim"], hidden, "relu"), (hidden, len(classes), "softmax")]
    parts = []
    for n_in, n_out, _ in dims:
        parts += [
            rng.normal(0, 1, (n_in, n_out)).astype("<f4"),
            rng.normal(0, 0.1, n_out).astype("<f4"),
        ]
    data = b"".join(p.tobytes() for p in parts)
    head = {
        "format": "bonic-head-v1",
        "baseModel": base,
        "backboneSha256": bb["backboneSha256"],
        "featureDim": bb["featureDim"],
        "classNames": classes,
        "layers": [
            {"kind": "dense", "in": i, "out": o, "activation": a} for i, o, a in dims
        ],
        "preprocess": {
            "size": bb["inputSize"],
            "fit": "stretch",
            "colorOrder": "RGB",
            "scale": "unit",
            "layout": "NHWC",
        },
        "featureNorm": "l2",
        "weightsFile": "head.bin",
        "weightsBytes": len(data),
        "weightsSha256": hashlib.sha256(data).hexdigest(),
    }
    if tweak:
        data = tweak(head, data) or data
    sha = hashlib.sha256(data).hexdigest()
    d = heads / sha
    d.mkdir(parents=True, exist_ok=True)
    (d / "head.json").write_text(json.dumps(head))
    (d / "head.bin").write_bytes(data)
    return sha


def _send(monkeypatch: pytest.MonkeyPatch, heads: Path, models: Dict[str, str]) -> None:
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    monkeypatch.setenv(_env.HEADS_DIR_ENV, str(heads))
    monkeypatch.setenv(_env.CUSTOM_MODELS_ENV, json.dumps(models))


def _frame(w: int = 640, h: int = 480, seed: int = 1) -> Any:
    return np.random.default_rng(seed).integers(0, 256, (h, w, 3), dtype=np.uint8)


# ── where it can run ─────────────────────────────────────────────────────


def test_importing_ai_loads_no_heavy_runtime() -> None:
    """`from bonicos import ai` must stay free — the simulator imports it."""
    code = (
        "import sys; from bonicos import ai; "
        "print('cv2' in sys.modules, 'mediapipe' in sys.modules)"
    )
    out = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=True
    )
    assert out.stdout.split() == ["False", "False"]


def test_in_the_browser_every_call_says_to_use_a_robot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sys, "platform", "emscripten")
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    for call in (
        lambda: ai.load("x"),
        lambda: ai.detect_objects(_frame()),
        ai.list_models,
    ):
        with pytest.raises(ai.AIUnavailable, match="not in the simulator"):
            call()


def test_in_the_browser_nothing_is_imported_before_the_check(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Pyodide has no numpy or OpenCV loaded. A function that imports them
    before checking where it runs dies with ModuleNotFoundError instead of
    the sentence — which 0.10.0's detectors did, in the real Pyodide."""
    monkeypatch.setattr(sys, "platform", "emscripten")
    for mod in ("numpy", "cv2", "mediapipe"):
        monkeypatch.setitem(sys.modules, mod, None)  # `import numpy` now raises
    calls = (
        lambda: ai.detect_objects(None),
        lambda: ai.detect_faces(None),
        lambda: ai.detect_markers(None),
        lambda: ai.detect_gestures(None),
        lambda: ai.load("x"),
        ai.list_models,
    )
    for call in calls:
        with pytest.raises(ai.AIUnavailable, match="not in the simulator"):
            call()


def test_off_robot_without_models_says_so() -> None:
    with pytest.raises(ai.AIUnavailable, match="runs on a robot"):
        ai.load("cup-detector")


@needs_models
def test_unreadable_model_list_is_reported(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _send(monkeypatch, tmp_path, {})
    monkeypatch.setenv(_env.CUSTOM_MODELS_ENV, "not json")
    with pytest.raises(ai.AIUnavailable, match="unreadable"):
        ai.load("x")
    monkeypatch.setenv(_env.CUSTOM_MODELS_ENV, json.dumps({"x": "not-a-sha"}))
    with pytest.raises(ai.AIUnavailable, match="bad entry"):
        ai.load("x")


# ── trained models ───────────────────────────────────────────────────────


@needs_models
def test_unknown_name_lists_what_was_sent(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sha = _write_head(tmp_path, ["cup", "bottle"])
    _send(monkeypatch, tmp_path, {"cup-detector": sha})
    with pytest.raises(ai.ModelNotFound) as e:
        ai.load("cup-dtector")
    assert '"cup-detector"' in str(e.value) and e.value.available == ["cup-detector"]
    assert ai.list_models() == ["cup-detector"]


@needs_models
def test_names_ignore_case_and_surrounding_space(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sha = _write_head(tmp_path, ["cup", "bottle"])
    _send(monkeypatch, tmp_path, {"Cup Detector": sha})
    assert ai.load("  cup   detector ").name == "Cup Detector"


@needs_models
def test_predict_ranks_every_class_and_sums_to_one(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sha = _write_head(tmp_path, ["cup", "bottle", "nothing"])
    _send(monkeypatch, tmp_path, {"cups": sha})
    model = ai.load("cups")
    result = model.predict(_frame())
    assert sorted(p.label for p in result) == ["bottle", "cup", "nothing"]
    assert [p.confidence for p in result] == sorted(
        (p.confidence for p in result), reverse=True
    )
    assert sum(p.confidence for p in result) == pytest.approx(1.0, abs=1e-5)
    label, confidence = result[0]  # a Prediction unpacks like a tuple
    assert model.classes == ["cup", "bottle", "nothing"]


@needs_models
def test_predict_follows_the_head_contract(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """RGB, whole frame stretched (a NON-square frame — §6.5), /255, NCHW into
    the backbone, L2-normalised features, then the head. Computed here
    independently of the SDK's own path."""
    sha = _write_head(tmp_path, ["a", "b", "c"], seed=3)
    _send(monkeypatch, tmp_path, {"m": sha})
    frame = _frame(640, 360, seed=7)

    net = cv2.dnn.readNetFromONNX(
        str(MODELS / _manifest()["backbones"]["mobilenet_v2_128"]["file"])
    )
    rgb = cv2.resize(frame[:, :, ::-1], (128, 128), interpolation=cv2.INTER_LINEAR)
    net.setInput((rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None])
    feats = net.forward().reshape(-1)
    feats = feats / (np.linalg.norm(feats) + 1e-12)
    data = (tmp_path / sha / "head.bin").read_bytes()
    v = np.frombuffer(data, "<f4")
    w1, b1 = v[: 1280 * 16].reshape(1280, 16), v[1280 * 16 : 1280 * 16 + 16]
    rest = v[1280 * 16 + 16 :]
    w2, b2 = rest[: 16 * 3].reshape(16, 3), rest[16 * 3 :]
    logits = np.maximum(feats @ w1 + b1, 0) @ w2 + b2
    expected = np.exp(logits - logits.max()) / np.exp(logits - logits.max()).sum()

    got = {p.label: p.confidence for p in ai.load("m").predict(frame)}
    assert [got[c] for c in "abc"] == pytest.approx(expected.tolist(), abs=1e-5)


@needs_models
def test_a_none_frame_explains_itself(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    sha = _write_head(tmp_path, ["a", "b"])
    _send(monkeypatch, tmp_path, {"m": sha})
    with pytest.raises(ValueError, match="has not delivered a frame"):
        ai.load("m").predict(None)


def _set(key: str, value: Any) -> Any:
    def tweak(head: Dict[str, Any], data: bytes) -> None:
        head[key] = value

    return tweak


@needs_models
@pytest.mark.parametrize(
    "tweak, match",
    [
        (_set("format", "bonic-head-v2"), "format"),
        (_set("backboneSha256", "0" * 64), "different build"),
        (_set("baseModel", "resnet50"), "does not have"),
        (_set("featureNorm", "l1"), "featureNorm"),
        (_set("classNames", ["only-one-for-three-outputs"]), "outputs 3 values"),
        (lambda h, d: h["preprocess"].update(fit="crop"), "preprocessing"),
        (lambda h, d: h["layers"][0].update(activation="tanh"), "activation"),
        (lambda h, d: h["layers"][0].update(out=100000), "invalid size"),
        (lambda h, d: h.update(weightsSha256="f" * 64), "checksum"),
        (lambda h, d: (h.update(weightsBytes=len(d) - 4), d[:-4])[1], "checksum"),
    ],
)
def test_bad_heads_are_refused_not_run(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, tweak: Any, match: str
) -> None:
    sha = _write_head(tmp_path, ["a", "b", "c"], tweak=tweak)
    _send(monkeypatch, tmp_path, {"m": sha})
    with pytest.raises(ai.InvalidModel, match=match):
        ai.load("m")


@needs_models
def test_truncated_weights_are_refused(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """A head.bin whose length disagrees with its layers, even with a matching
    checksum, never reaches np.frombuffer's reshape."""

    def truncate(head: Dict[str, Any], data: bytes) -> bytes:
        short = data[:-4]
        head.update(
            weightsBytes=len(short), weightsSha256=hashlib.sha256(short).hexdigest()
        )
        return short

    sha = _write_head(tmp_path, ["a", "b", "c"], tweak=truncate)
    _send(monkeypatch, tmp_path, {"m": sha})
    with pytest.raises(ai.InvalidModel, match="layers need"):
        ai.load("m")


@needs_models
def test_nan_weights_are_refused(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    def poison(head: Dict[str, Any], data: bytes) -> bytes:
        bad = np.frombuffer(data, "<f4").copy()
        bad[5] = np.nan
        out = bad.tobytes()
        head.update(weightsSha256=hashlib.sha256(out).hexdigest())
        return out

    sha = _write_head(tmp_path, ["a", "b"], tweak=poison)
    _send(monkeypatch, tmp_path, {"m": sha})
    with pytest.raises(ai.InvalidModel, match="NaN"):
        ai.load("m")


@needs_models
def test_missing_head_files_ask_for_a_resend(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _send(monkeypatch, tmp_path, {"m": "a" * 64})
    with pytest.raises(ai.InvalidModel, match="Run the program again"):
        ai.load("m")


# ── built-in detectors ───────────────────────────────────────────────────


@needs_models
def test_markers_are_found_with_their_id_and_centre(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    frame = np.full((480, 640, 3), 255, np.uint8)
    frame[200:360, 300:460] = cv2.cvtColor(
        cv2.aruco.generateImageMarker(d, 7, 160), cv2.COLOR_GRAY2BGR
    )
    (marker,) = ai.detect_markers(frame)
    assert marker.id == 7
    assert marker.center == pytest.approx((380, 280), abs=2)


@needs_models
def test_detectors_find_nothing_in_a_blank_frame(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    blank = np.zeros((480, 640, 3), np.uint8)
    assert ai.detect_objects(blank) == []
    assert ai.detect_objects(blank, input_size=256) == []
    assert ai.detect_faces(blank) == []
    assert ai.detect_markers(blank) == []


@needs_models
def test_object_input_size_is_checked(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    with pytest.raises(ValueError, match="input_size"):
        ai.detect_objects(_frame(), input_size=300)


@needs_models
def test_gestures_without_mediapipe_say_why(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(_env.MODELS_DIR_ENV, str(MODELS))
    monkeypatch.setitem(sys.modules, "mediapipe", None)
    with pytest.raises(ai.AIUnavailable, match="MediaPipe"):
        ai.detect_gestures(_frame())


def test_frames_are_checked_before_any_model_runs() -> None:
    with pytest.raises(ValueError, match="height x width x 3"):
        ai.detect_markers(np.zeros((4, 4, 5), np.uint8))
    with pytest.raises(ValueError, match="uint8"):
        ai.detect_markers(np.zeros((4, 4, 3), np.float32))
