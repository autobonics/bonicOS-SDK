"""Navigation, mapping & named locations (API.md §4).

Fire-and-monitor: goal methods start navigation and get an immediate
``ack {goal_id}``; real completion arrives as ``nav_status`` telemetry
events (PROTOCOL.md §4 pattern 2). ``wait_for_goal`` watches that stream.
"""

from __future__ import annotations

import base64
import time
import zlib
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .. import protocol
from ._base import ControllerBase

_TERMINAL_STATUSES = frozenset({"succeeded", "failed", "canceled"})


class NavigationController(ControllerBase):
    def go_to(
        self,
        x: float,
        y: float,
        theta: float = 0.0,
        wait: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        self._require_feature("navigation")
        self._command({"type": protocol.CMD_NAV_GOAL, "x": x, "y": y, "theta": theta})
        if not wait:
            return True
        return self.wait_for_goal(timeout)

    def navigate_waypoints(
        self,
        points: Sequence[Tuple[float, ...]],
        wait: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        self._require_feature("navigation")
        waypoints = []
        for point in points:
            x, y = point[0], point[1]
            wp: Dict[str, float] = {"x": x, "y": y}
            if len(point) > 2:
                wp["theta"] = point[2]
            waypoints.append(wp)
        self._command(
            {
                "type": protocol.CMD_NAVIGATE_THROUGH_WAYPOINTS,
                "waypoints": waypoints,
            }
        )
        if not wait:
            return True
        return self.wait_for_goal(timeout)

    def cancel_goal(self) -> bool:
        result = self._command({"type": protocol.CMD_CANCEL_NAV})
        return bool(result.get("canceled", False))

    def wait_for_goal(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while True:
            status = self.get_nav_status()
            if status in _TERMINAL_STATUSES:
                return status == "succeeded"
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._transport.wait_for_update(min(remaining, 1.0))

    def get_nav_status(self) -> str:
        event = self._latest(protocol.EVENT_NAV_STATUS)
        return event.get("status", "idle") if event else "idle"

    def get_distance_to_goal(self) -> float:
        event = self._latest(protocol.EVENT_NAV_STATUS)
        return float(event.get("distance_to_goal", 0.0)) if event else 0.0

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0) -> bool:
        result = self._command(
            {"type": protocol.CMD_SET_INITIAL_POSE, "x": x, "y": y, "theta": theta}
        )
        return bool(result.get("ok", False))

    def start_navigation(self) -> bool:
        result = self._command({"type": protocol.CMD_START_NAVIGATION})
        return bool(result.get("ok", False))

    def stop_navigation(self) -> bool:
        result = self._command({"type": protocol.CMD_STOP_NAVIGATION})
        return bool(result.get("ok", False))

    # --- nav mode (mapping <-> navigation session switching) ---------------
    #
    # Distinct from start_mapping/stop_mapping below: these bring up or tear
    # down the WHOLE ROS launch tree (slam_toolbox+Nav2, or map_server+AMCL+
    # Nav2), while start_mapping/stop_mapping just pause/unpause slam_toolbox
    # inside an already-entered mapping session. Typical mapping workflow:
    # enter_mapping_mode() -> start_mapping() -> drive around -> stop_mapping()
    # -> save_map(name) -> enter_navigation_mode(name).
    #
    # Session bring-up/teardown launches a ROS process tree and waits for it
    # to settle, so these are slow (several seconds) — much longer than the
    # 5s default command timeout other methods use.

    def enter_mapping_mode(self, timeout: float = 30.0) -> bool:
        """Tear down any navigation session and launch slam_toolbox+Nav2."""
        self._require_feature("mapping")
        result = self._command(
            {"type": protocol.CMD_ENTER_MAPPING_MODE}, timeout=timeout
        )
        return bool(result.get("ok", False))

    def enter_navigation_mode(self, name: str, timeout: float = 30.0) -> bool:
        """Tear down any mapping session and launch map_server+AMCL+Nav2
        localizing on the saved map ``name``. False if the map doesn't exist
        or the session fails to come up."""
        self._require_feature("navigation")
        result = self._command(
            {"type": protocol.CMD_ENTER_NAVIGATION_MODE, "name": name}, timeout=timeout
        )
        return bool(result.get("ok", False))

    def stop_nav_mode(self, timeout: float = 15.0) -> bool:
        """Tear down the current nav session -> idle. Base drive/sensors
        stay up; only the mapping/navigation launch tree is killed."""
        result = self._command({"type": protocol.CMD_STOP_NAV_MODE}, timeout=timeout)
        return bool(result.get("ok", False))

    def get_nav_mode(self) -> Dict[str, Any]:
        """Current session state: ``{"mode", "map", "transitioning",
        "localized"}``.

        Issues ``CMD_GET_NAV_MODE`` directly rather than reading cached
        ``nav_mode`` telemetry, so it's always a fresh, synchronous read —
        useful right after connecting, before any transition telemetry has
        arrived. ``localized`` is freshness-checked, not latched — a robot can
        be ``navigating`` and still report ``localized: False`` if AMCL's seed
        hasn't landed yet or it later loses the pose (e.g. after a base-stack
        restart); goals will fail to plan until it's True. If it stays False,
        call ``set_initial_pose`` to place the robot by hand.
        """
        result = self._command({"type": protocol.CMD_GET_NAV_MODE})
        return {
            "mode": result.get("mode", "idle"),
            "map": result.get("map"),
            "transitioning": bool(result.get("transitioning", False)),
            "localized": bool(result.get("localized", False)),
        }

    # --- mapping -----------------------------------------------------------

    def start_mapping(self) -> bool:
        result = self._command({"type": protocol.CMD_START_MAPPING})
        return bool(result.get("ok", False))

    def stop_mapping(self) -> bool:
        result = self._command({"type": protocol.CMD_STOP_MAPPING})
        return bool(result.get("ok", False))

    def save_map(self, name: str = "map") -> bool:
        result = self._command({"type": protocol.CMD_SAVE_MAP, "name": name})
        return bool(result.get("ok", False))

    def load_map(self, name: str) -> bool:
        """Swap the map a running navigation session localizes against.

        This is the fast in-place path (nav2 map_server's ``/load_map``
        service) — it only works while already in navigation mode (see
        ``enter_navigation_mode``); False if map_server isn't up (e.g. still
        in mapping mode) or the map doesn't exist. The server auto-reseeds
        AMCL against the new map on success (the last pose remembered on it,
        or its origin if never visited) — check ``get_nav_mode()["localized"]``
        rather than assuming it landed; a slow/loaded host can still miss the
        seeding deadline.
        """
        result = self._command({"type": protocol.CMD_LOAD_MAP, "name": name})
        return bool(result.get("ok", False))

    def delete_map(self, name: str) -> bool:
        """Delete a saved map and its sidecar files.

        False if ``name`` doesn't exist or is the map a live navigation
        session is currently localized against — stop that session
        (``stop_nav_mode``) or switch it to a different map first.
        """
        result = self._command({"type": protocol.CMD_DELETE_MAP, "name": name})
        return bool(result.get("ok", False))

    def list_maps(self) -> List[str]:
        """Saved map names.

        The server's ``maps`` list is actually a list of metadata dicts
        (``{"name", "size", "modified"}`` — ``bonicOS-robot-app``'s
        ``MapManager.list()``, verified against the real M1 sim
        2026-08-04), not plain strings as this method's contract promises.
        Extract just the name — a student wants ``load_map(name)``-ready
        values, not to guess the server's internal metadata shape.
        """
        result = self._command({"type": protocol.CMD_LIST_MAPS})
        return [
            entry["name"] if isinstance(entry, dict) else entry
            for entry in result.get("maps", [])
        ]

    def get_map(self) -> Optional[Dict[str, Any]]:
        """Latest occupancy grid (decoded from the cached ``map`` event)."""
        return self._decode_grid(self._latest(protocol.EVENT_MAP))

    def get_costmap(self) -> Optional[Dict[str, Any]]:
        return self._decode_grid(self._latest(protocol.EVENT_COSTMAP))

    def get_plan(self) -> List[Tuple[float, float]]:
        event = self._latest(protocol.EVENT_PLAN)
        if not event:
            return []
        return [(p[0], p[1]) for p in event.get("points", [])]

    @staticmethod
    def _decode_grid(event: Optional[dict]) -> Optional[Dict[str, Any]]:
        if not event:
            return None
        info = event.get("info", {})
        data_b64 = event.get("data_b64")
        data = zlib.decompress(base64.b64decode(data_b64)) if data_b64 else b""
        return {"info": info, "data": data}

    # --- named locations (all 🔌 stub in v1, PROTOCOL.md §5.3) -------------

    def save_location(self, name: str) -> bool:
        result = self._command({"type": protocol.CMD_SAVE_LOCATION, "name": name})
        return bool(result.get("ok", False))

    def goto_location(
        self, name: str, wait: bool = True, timeout: float = 60.0
    ) -> bool:
        self._command({"type": protocol.CMD_GOTO_LOCATION, "name": name})
        if not wait:
            return True
        return self.wait_for_goal(timeout)

    def list_locations(self) -> List[str]:
        result = self._command({"type": protocol.CMD_LIST_LOCATIONS})
        return list(result.get("locations", []))

    def delete_location(self, name: str) -> bool:
        result = self._command({"type": protocol.CMD_DELETE_LOCATION, "name": name})
        return bool(result.get("ok", False))

    def delete_all_locations(self) -> bool:
        result = self._command({"type": protocol.CMD_DELETE_ALL_LOCATIONS})
        return bool(result.get("ok", False))
