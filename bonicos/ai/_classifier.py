"""Trained models — ``ai.load(name).predict(frame)``.

A trained model is a small head (``bonic-head-v1``) on top of a shared
MobileNetV2 backbone that the robot image already carries. The browser trains
the head and writes it; this module reads it. The format is a cross-runtime
contract, specified in AITrainingAndInferenceOverview.md §6, and every rule
below comes from there:

* preprocessing (§6.3, §6.5): BGR → **RGB**, the whole frame **stretched** to
  ``size × size`` (no crop), pixels / 255. Getting any of these wrong does not
  raise; it gives confident nonsense that looks like a badly trained model.
* ``featureNorm: "l2"`` (§6.1): the 1280-d feature vector is scaled to unit
  length before the head. Absent means ``"none"``.
* ``head.bin`` (§6.2): little-endian float32, per dense layer the ``[in][out]``
  row-major weights then the ``out`` biases, no header.
* ``backboneSha256`` (§6.4): a head is bound to one backbone *build*, not a
  name. A mismatch is refused, never run.

Heads arrive from a browser, so they are validated before any array is built
from them: sizes are capped and the byte count must equal what the layers
imply. ``np.frombuffer`` only — never pickle.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any, Dict, List, Tuple

from ..exceptions import InvalidModel, ModelNotFound
from . import _env
from ._types import Prediction, as_bgr, cv2_module

HEAD_FORMAT = "bonic-head-v1"
_MAX_LAYERS = 4
_MAX_DIM = 4096
_MAX_CLASSES = 100
_MAX_HEAD_JSON = 64 * 1024

# One backbone per variant for the life of the process: each is ~9 MB and
# ~150 ms to load, and every model trained against it shares it.
_backbones: Dict[str, Any] = {}
_models: Dict[Tuple[str, str], "Model"] = {}


class Model:
    """A trained model loaded with :func:`bonicos.ai.load`."""

    def __init__(
        self,
        name: str,
        head: Dict[str, Any],
        layers: List[Tuple[Any, Any, str]],
        backbone: Dict[str, Any],
    ) -> None:
        self.name = name
        self._head = head
        self._layers = layers
        self._backbone = backbone
        self._size = int(head["preprocess"]["size"])
        self._l2 = head.get("featureNorm", "none") == "l2"

    @property
    def classes(self) -> List[str]:
        """The class names, in the order they were trained."""
        return list(self._head["classNames"])

    def __repr__(self) -> str:
        return f"<ai.Model {self.name!r} classes={self.classes}>"

    def features(self, frame: Any) -> Any:
        """The backbone's 1280-d feature vector for ``frame``, before any
        normalisation. Exposed for testing and for curious students."""
        import numpy as np

        cv2 = cv2_module()
        rgb = cv2.cvtColor(as_bgr(frame), cv2.COLOR_BGR2RGB)
        x = cv2.resize(rgb, (self._size, self._size), interpolation=cv2.INTER_LINEAR)
        blob = (x.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
        net = _backbone_net(self._backbone)
        net.setInput(blob)
        return net.forward().reshape(-1)

    def predict(self, frame: Any) -> List[Prediction]:
        """Every class with its confidence, most likely first —
        ``[Prediction("cup", 0.94), Prediction("bottle", 0.05), …]``."""
        import numpy as np

        feats = self.features(frame).astype(np.float32)
        if self._l2:
            feats = feats / (np.linalg.norm(feats) + 1e-12)
        probs = run_head(feats, self._layers)
        order = np.argsort(-probs)
        names = self._head["classNames"]
        return [Prediction(names[i], float(probs[i])) for i in order]


def run_head(feats: Any, layers: List[Tuple[Any, Any, str]]) -> Any:
    """The head's forward pass. Dropout is training-only and absent here."""
    import numpy as np

    x = feats
    for w, b, activation in layers:
        x = x @ w + b
        if activation == "relu":
            x = np.maximum(x, 0.0)
        elif activation == "softmax":
            e = np.exp(x - x.max())
            x = e / e.sum()
    return x


def list_models() -> List[str]:
    """Names of the trained models sent with this program."""
    _env.manifest()  # the same "can AI run here at all" check load() makes
    return sorted(name for name, _ in _env.custom_models().values())


def load(name: str) -> Model:
    """A trained model by the name it has in the Train tab. Case and
    surrounding spaces are ignored."""
    root, manifest = _env.manifest()
    sent = _env.custom_models()
    key = _env.normalize_name(name)
    if key not in sent:
        raise ModelNotFound(name, [n for n, _ in sent.values()])
    display, sha = sent[key]
    if (sha, display) in _models:
        return _models[(sha, display)]

    head_dir = _env.heads_dir() / sha
    json_path, bin_path = head_dir / "head.json", head_dir / "head.bin"
    if not json_path.is_file() or not bin_path.is_file():
        raise InvalidModel(
            display,
            "its files are missing on this robot. Run the program again to resend it.",
        )
    if json_path.stat().st_size > _MAX_HEAD_JSON:
        raise InvalidModel(display, "head.json is too large to be a model description.")
    try:
        head = json.loads(json_path.read_text())
    except ValueError:
        raise InvalidModel(display, "head.json is not valid JSON.") from None
    if not isinstance(head, dict):
        raise InvalidModel(display, "head.json is not a model description.")

    backbone = _check_head(display, head, manifest)
    data = bin_path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != sha or digest != head.get("weightsSha256"):
        raise InvalidModel(
            display,
            "head.bin does not match its checksum — it was damaged in transfer.",
        )
    layers = _unpack(display, head, data)

    backbone = dict(backbone, _root=str(root))
    model = Model(display, head, layers, backbone)
    _models[(sha, display)] = model
    return model


def _check_head(
    name: str, head: Dict[str, Any], manifest: Dict[str, Any]
) -> Dict[str, Any]:
    """Validate everything about a head except its weights; return the manifest
    entry of the backbone it needs."""

    def bad(reason: str) -> InvalidModel:
        return InvalidModel(name, reason)

    if head.get("format") != HEAD_FORMAT:
        raise bad(
            f"it is in format {head.get('format')!r}; this version of bonicos "
            f"reads {HEAD_FORMAT!r}."
        )
    base = head.get("baseModel")
    backbone = manifest["backbones"].get(base) if isinstance(base, str) else None
    if backbone is None:
        raise bad(f"it was trained on {base!r}, which this robot does not have.")
    if head.get("backboneSha256") != backbone["backboneSha256"]:
        raise bad(
            f"it was trained against a different build of {base} than this robot "
            "carries. "
            "Retrain it in the Train tab."
        )
    if head.get("featureDim") != backbone["featureDim"]:
        raise bad(
            f"featureDim is {head.get('featureDim')!r}, but {base} produces "
            f"{backbone['featureDim']}."
        )

    pre = head.get("preprocess")
    expected = {
        "size": backbone["inputSize"],
        "fit": "stretch",
        "colorOrder": "RGB",
        "scale": "unit",
    }
    if not isinstance(pre, dict) or any(pre.get(k) != v for k, v in expected.items()):
        raise bad(
            f"it asks for preprocessing {pre!r}; this robot implements {expected!r}."
        )
    if head.get("featureNorm", "none") not in ("none", "l2"):
        raise bad(f"featureNorm {head.get('featureNorm')!r} is not supported.")

    classes = head.get("classNames")
    if (
        not isinstance(classes, list)
        or not 1 <= len(classes) <= _MAX_CLASSES
        or not all(isinstance(c, str) for c in classes)
    ):
        raise bad("its class names are missing or malformed.")

    layers = head.get("layers")
    if not isinstance(layers, list) or not 1 <= len(layers) <= _MAX_LAYERS:
        raise bad(f"it must have 1 to {_MAX_LAYERS} layers.")
    width = backbone["featureDim"]
    for i, layer in enumerate(layers):
        last = i == len(layers) - 1
        if not isinstance(layer, dict) or layer.get("kind") != "dense":
            raise bad(f"layer {i} is not a dense layer.")
        n_in, n_out = layer.get("in"), layer.get("out")
        if not (
            isinstance(n_in, int) and isinstance(n_out, int) and 1 <= n_out <= _MAX_DIM
        ):
            raise bad(f"layer {i} has an invalid size.")
        if n_in != width:
            raise bad(f"layer {i} expects {n_in} inputs but receives {width}.")
        if layer.get("activation") != ("softmax" if last else "relu"):
            raise bad(f"layer {i} uses activation {layer.get('activation')!r}.")
        width = n_out
    if width != len(classes):
        raise bad(f"it outputs {width} values for {len(classes)} classes.")
    return dict(backbone)


def _unpack(name: str, head: Dict[str, Any], data: bytes) -> List[Tuple[Any, Any, str]]:
    import numpy as np

    floats = sum(layer["in"] * layer["out"] + layer["out"] for layer in head["layers"])
    if len(data) != 4 * floats or head.get("weightsBytes") != len(data):
        raise InvalidModel(
            name, f"head.bin is {len(data)} bytes; its layers need {4 * floats}."
        )
    values = np.frombuffer(data, dtype="<f4")
    out: List[Tuple[Any, Any, str]] = []
    offset = 0
    for layer in head["layers"]:
        n_in, n_out = layer["in"], layer["out"]
        w = values[offset : offset + n_in * n_out].reshape(n_in, n_out)
        offset += n_in * n_out
        b = values[offset : offset + n_out]
        offset += n_out
        out.append((w, b, layer["activation"]))
    if not all(np.isfinite(w).all() and np.isfinite(b).all() for w, b, _ in out):
        raise InvalidModel(name, "its weights contain NaN or infinity.")
    return out


def _backbone_net(entry: Dict[str, Any]) -> Any:
    path = f"{entry['_root']}/{entry['file']}"
    if path not in _backbones:
        _backbones[path] = cv2_module().dnn.readNetFromONNX(path)
    return _backbones[path]
