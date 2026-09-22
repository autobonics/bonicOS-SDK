"""Where ``bonicos.ai`` finds its models — the one module that reads the environment.

Two sources, both set by robot_app for each ``run_code`` run:

``$BONICOS_MODELS_DIR``
    The bundled models baked into the robot image (``/opt/bonicos/models``):
    the MobileNet backbones and the built-in detectors, described by
    ``manifest.json`` (format ``bonicos-models-v1``).

``$BONICOS_AI_MODELS`` + ``$BONICOS_AI_HEADS_DIR``
    The trained models sent with THIS program. ``BONICOS_AI_MODELS`` is a JSON
    object ``{name: sha256}``; each head lives at
    ``$BONICOS_AI_HEADS_DIR/<sha256>/head.json`` + ``head.bin``, where the
    sha256 is that of ``head.bin``. The map is per run, never a robot-wide
    index: two students' models may share a name and must not collide
    (AITrainingAndInferenceOverview.md §14.0).

Nothing is imported or read at module import time, so ``from bonicos import
ai`` is free — and succeeds in the browser simulator, where every call raises
:class:`AIUnavailable` with a sentence instead.
"""

from __future__ import annotations

import json
import os
import re
import sys
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Tuple

from ..exceptions import AIUnavailable

MODELS_DIR_ENV = "BONICOS_MODELS_DIR"
CUSTOM_MODELS_ENV = "BONICOS_AI_MODELS"
HEADS_DIR_ENV = "BONICOS_AI_HEADS_DIR"

MANIFEST_FORMAT = "bonicos-models-v1"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def normalize_name(name: str) -> str:
    """How model names are compared everywhere: surrounding whitespace dropped,
    inner runs of whitespace collapsed, case ignored. The browser's pre-run
    check must use the same rule, or it passes names the robot then rejects."""
    return " ".join(str(name).split()).casefold()


def check_runtime() -> None:
    """Raise :class:`AIUnavailable` in the browser simulator.

    Every public ``ai`` function calls this (directly, or through
    :func:`models_dir`) BEFORE importing anything. Pyodide has not loaded numpy
    or OpenCV, so a function that imported first would fail there with a bare
    ``ModuleNotFoundError`` instead of this sentence — found running 0.10.0 in
    the real Pyodide runtime.
    """
    if sys.platform == "emscripten":
        raise AIUnavailable(
            "AI models run on a robot, not in the simulator. "
            "Switch the run target to a robot to use them."
        )


def models_dir() -> Path:
    """The bundled models directory, or :class:`AIUnavailable` saying why not."""
    check_runtime()
    raw = os.environ.get(MODELS_DIR_ENV)
    if not raw:
        raise AIUnavailable(
            "AI models are only available when your program runs on a robot "
            f"(${MODELS_DIR_ENV} is not set)."
        )
    path = Path(raw)
    if not (path / "manifest.json").is_file():
        raise AIUnavailable(
            f"no AI models were found at {path} — this robot's software may need "
            "updating."
        )
    return path


@lru_cache(maxsize=4)
def _read_manifest(path: str) -> Dict[str, Any]:
    data: Dict[str, Any] = json.loads((Path(path) / "manifest.json").read_text())
    if data.get("format") != MANIFEST_FORMAT:
        raise AIUnavailable(
            f"this robot's AI models are in format {data.get('format')!r}, which "
            f"this version of bonicos does not read (it expects {MANIFEST_FORMAT!r})."
        )
    return data


def manifest() -> Tuple[Path, Dict[str, Any]]:
    """``(models_dir, parsed manifest.json)``."""
    root = models_dir()
    return root, _read_manifest(str(root))


def custom_models() -> Dict[str, Tuple[str, str]]:
    """The trained models sent with this program, as
    ``{normalized name: (name as sent, sha256)}``. Empty when none were sent."""
    raw = os.environ.get(CUSTOM_MODELS_ENV, "")
    if not raw:
        return {}
    try:
        data = json.loads(raw)
    except ValueError:
        data = None
    if not isinstance(data, dict):
        raise AIUnavailable(
            f"the list of AI models sent with this program is unreadable "
            f"(${CUSTOM_MODELS_ENV} is not a JSON object)."
        )
    out: Dict[str, Tuple[str, str]] = {}
    for name, sha in data.items():
        if (
            not isinstance(name, str)
            or not isinstance(sha, str)
            or not _SHA256.match(sha)
        ):
            raise AIUnavailable(
                f"the list of AI models sent with this program has a bad entry "
                f"for {name!r}."
            )
        out[normalize_name(name)] = (name, sha)
    return out


def heads_dir() -> Path:
    raw = os.environ.get(HEADS_DIR_ENV)
    if not raw:
        raise AIUnavailable(
            f"trained models were listed but ${HEADS_DIR_ENV} is not set — the "
            "robot did not say where it stored them."
        )
    return Path(raw)
