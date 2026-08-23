"""Validates the three preset worlds from `SimulatorPresetWorlds.md` against
the real planner, at all three product-series clearances.

The presets themselves live in the frontend
(`bonicAI-frontend/src/lib/code-studio/presetWorlds.ts`, TypeScript, not
importable here) — this file mirrors their obstacle tables (SimulatorPresetWorlds.md §4)
as the honest, visible-duplication answer to that cross-repo split (§6, option
3). If you change a coordinate in one place, change it in the other and
re-run this file; a silent mismatch is a world that works for one series and
fails for another with no visible reason in the student's `go_to()`.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import pytest

from bonicos.transports.sim import SimTransport

# Mirrors productCatalog.ts's `SERIES_SIM_PROFILE[*].baseRadius` — the three
# `radius=` values `SimTransport` is constructed with per series.
SERIES_BASE_RADIUS: Dict[str, float] = {
    "a": 0.16,
    "s": 0.19,
    "m": 0.22,
}


def _wall(id_: str, x: float, y: float, size_x: float, size_y: float) -> dict:
    return {"id": id_, "x": x, "y": y, "sizeX": size_x, "sizeY": size_y}


# Mirrors presetWorlds.ts's "open-room" — room x in [-1, 9], y in [-4, 4].
OPEN_ROOM_OBSTACLES: List[dict] = [
    _wall("wall-n", 4, 4, 10.2, 0.2),
    _wall("wall-s", 4, -4, 10.2, 0.2),
    _wall("wall-w", -1, 0, 0.2, 8.2),
    _wall("wall-e", 9, 0, 0.2, 8.2),
    _wall("crate-1", 2, 2, 0.6, 0.6),
    _wall("crate-2", 4, -2, 0.6, 0.6),
    _wall("crate-3", 6, 1.5, 0.6, 0.6),
    _wall("crate-4", 1.5, -2.5, 0.5, 0.5),
    _wall("pillar-1", 3, -0.5, 0.4, 0.4),
    _wall("pillar-2", 7, -2.5, 0.4, 0.4),
    _wall("pillar-3", 6.5, 3, 0.4, 0.4),
]
OPEN_ROOM_GOAL: Tuple[float, float] = (8.0, -3.0)

# Mirrors presetWorlds.ts's "doorways" — room x in [-1, 11], y in [-4, 4],
# divided at x=3, x=6, x=9.
DOORWAYS_OBSTACLES: List[dict] = [
    _wall("wall-n", 5, 4, 12.2, 0.2),
    _wall("wall-s", 5, -4, 12.2, 0.2),
    _wall("wall-w", -1, 0, 0.2, 8.2),
    _wall("wall-e", 11, 0, 0.2, 8.2),
    _wall("div1-lower", 3, -2.25, 0.2, 3.5),
    _wall("div1-upper", 3, 2.25, 0.2, 3.5),
    _wall("div2-lower", 6, -1.0, 0.2, 6.0),
    _wall("div2-upper", 6, 3.5, 0.2, 1.0),
    _wall("div3-lower", 9, -3.5, 0.2, 1.0),
    _wall("div3-upper", 9, 1.0, 0.2, 6.0),
]
DOORWAYS_GOAL: Tuple[float, float] = (10.0, 0.0)

# Mirrors presetWorlds.ts's "maze" — room x in [-1, 11], y in [-3.2, 3.2],
# three alternating baffles.
MAZE_OBSTACLES: List[dict] = [
    _wall("wall-n", 5, 3.2, 12.2, 0.2),
    _wall("wall-s", 5, -3.2, 12.2, 0.2),
    _wall("wall-w", -1, 0, 0.2, 6.6),
    _wall("wall-e", 11, 0, 0.2, 6.6),
    _wall("baffle-1", 4.3, 1.6, 10.6, 0.2),
    _wall("baffle-2", 5.7, 0.0, 10.6, 0.2),
    _wall("baffle-3", 4.3, -1.6, 10.6, 0.2),
]
MAZE_GOAL: Tuple[float, float] = (0.0, -2.4)

PRESET_WORLDS: Dict[str, Tuple[List[dict], Tuple[float, float]]] = {
    "open-room": (OPEN_ROOM_OBSTACLES, OPEN_ROOM_GOAL),
    "doorways": (DOORWAYS_OBSTACLES, DOORWAYS_GOAL),
    "maze": (MAZE_OBSTACLES, MAZE_GOAL),
}


@pytest.mark.parametrize("world_id", sorted(PRESET_WORLDS))
@pytest.mark.parametrize("series", sorted(SERIES_BASE_RADIUS))
def test_preset_world_is_navigable(world_id: str, series: str) -> None:
    obstacles, goal = PRESET_WORLDS[world_id]
    sim = SimTransport(obstacles=obstacles, radius=SERIES_BASE_RADIUS[series])

    assert not sim._collides(0.0, 0.0), (
        f"{world_id}: start pose (0, 0) collides for series {series!r}"
    )
    assert not sim._collides(*goal), (
        f"{world_id}: suggested goal {goal} collides for series {series!r}"
    )
    path = sim._plan_path((0.0, 0.0), goal)
    assert path is not None, (
        f"{world_id}: no path from (0, 0) to {goal} for series {series!r}"
    )
