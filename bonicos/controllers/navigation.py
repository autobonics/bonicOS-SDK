"""Navigation, mapping, named locations & docking (API.md §4).

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
        result = self._command(
            {"type": protocol.CMD_NAV_GOAL, "x": x, "y": y, "theta": theta}
        )
        if not wait:
            return True
        return self.wait_for_goal(timeout, goal_id=result.get("goal_id"))

    def navigate_waypoints(
        self,
        points: Sequence[Tuple[float, ...]],
        wait: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        waypoints = []
        for point in points:
            x, y = point[0], point[1]
            wp: Dict[str, float] = {"x": x, "y": y}
            if len(point) > 2:
                wp["theta"] = point[2]
            waypoints.append(wp)
        result = self._command(
            {
                "type": protocol.CMD_NAVIGATE_THROUGH_WAYPOINTS,
                "waypoints": waypoints,
            }
        )
        if not wait:
            return True
        return self.wait_for_goal(timeout, goal_id=result.get("goal_id"))

    def cancel_goal(self) -> bool:
        result = self._command({"type": protocol.CMD_CANCEL_NAV})
        return bool(result.get("canceled", False))

    def wait_for_goal(
        self, timeout: float = 30.0, goal_id: Optional[str] = None
    ) -> bool:
        """Block until the active goal reaches a terminal status.

        ``goal_id`` is the id the goal's own ack carried (PROTOCOL.md §5.2's
        ``ack {goal_id}``), and the goal methods pass it automatically. It
        matters because ``nav_status`` is *cached* telemetry that outlives
        the goal that produced it: a robot acks a new goal immediately, but
        the first ``navigating`` event only arrives once Nav2's action server
        accepts it (bonicOS-robot-app ``ros/nav_client.py``). Without an id to
        match, the second and every later ``go_to(wait=True)`` reads the
        *previous* goal's cached ``succeeded`` and returns True instantly,
        before the robot has moved.

        An event carrying no ``goal_id`` at all still matches — a stub server
        that never sets one is no worse off than before.
        """
        return self._wait_for_terminal(protocol.EVENT_NAV_STATUS, timeout, goal_id)

    def _wait_for_terminal(
        self, event_name: str, timeout: float, goal_id: Optional[str]
    ) -> bool:
        """Shared by ``wait_for_goal`` and ``wait_for_dock``: both events
        carry the same ``goal_id`` and status vocabulary."""
        deadline = time.monotonic() + timeout
        while True:
            event = self._latest(event_name)
            if event is not None and self._is_for_goal(event, goal_id):
                status = event.get("status", "idle")
                if status in _TERMINAL_STATUSES:
                    return status == "succeeded"
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._transport.wait_for_update(min(remaining, 1.0))

    @staticmethod
    def _is_for_goal(event: dict, goal_id: Optional[str]) -> bool:
        if goal_id is None:
            return True
        event_goal_id = event.get("goal_id")
        return event_goal_id is None or str(event_goal_id) == str(goal_id)

    def get_nav_status(self) -> str:
        event = self._latest(protocol.EVENT_NAV_STATUS)
        return event.get("status", "idle") if event else "idle"

    def get_distance_to_goal(self) -> float:
        event = self._latest(protocol.EVENT_NAV_STATUS)
        return float(event.get("distance_to_goal", 0.0)) if event else 0.0

    def set_initial_pose(self, x: float, y: float, theta: float = 0.0) -> bool:
        self._command(
            {"type": protocol.CMD_SET_INITIAL_POSE, "x": x, "y": y, "theta": theta}
        )
        return True

    def start_navigation(self) -> bool:
        self._command({"type": protocol.CMD_START_NAVIGATION})
        return True

    def stop_navigation(self) -> bool:
        self._command({"type": protocol.CMD_STOP_NAVIGATION})
        return True

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
        self._command(
            {"type": protocol.CMD_ENTER_MAPPING_MODE}, timeout=timeout
        )
        return True

    def enter_navigation_mode(self, name: str, timeout: float = 30.0) -> bool:
        """Tear down any mapping session and launch map_server+AMCL+Nav2
        localizing on the saved map ``name``. Raises
        :class:`~bonicos.CommandError` if the map doesn't exist or the session
        fails to come up."""
        self._command(
            {"type": protocol.CMD_ENTER_NAVIGATION_MODE, "name": name}, timeout=timeout
        )
        return True

    def stop_nav_mode(self, timeout: float = 15.0) -> bool:
        """Tear down the current nav session -> idle. Base drive/sensors
        stay up; only the mapping/navigation launch tree is killed."""
        self._command({"type": protocol.CMD_STOP_NAV_MODE}, timeout=timeout)
        return True

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
        self._command({"type": protocol.CMD_START_MAPPING})
        return True

    def stop_mapping(self) -> bool:
        self._command({"type": protocol.CMD_STOP_MAPPING})
        return True

    def save_map(self, name: str = "map") -> bool:
        self._command({"type": protocol.CMD_SAVE_MAP, "name": name})
        return True

    def load_map(self, name: str) -> bool:
        """Swap the map a running navigation session localizes against.

        This is the fast in-place path (nav2 map_server's ``/load_map``
        service) — it only works while already in navigation mode (see
        ``enter_navigation_mode``); raises :class:`~bonicos.CommandError` if
        map_server isn't up (e.g. still in mapping mode) or the map doesn't
        exist. The server auto-reseeds
        AMCL against the new map on success (the last pose remembered on it,
        or its origin if never visited) — check ``get_nav_mode()["localized"]``
        rather than assuming it landed; a slow/loaded host can still miss the
        seeding deadline.
        """
        self._command({"type": protocol.CMD_LOAD_MAP, "name": name})
        return True

    def delete_map(self, name: str) -> bool:
        """Delete a saved map and its sidecar files.

        Raises :class:`~bonicos.CommandError` if ``name`` doesn't exist or is
        the map a live navigation session is currently localized against — stop that session
        (``stop_nav_mode``) or switch it to a different map first.
        """
        self._command({"type": protocol.CMD_DELETE_MAP, "name": name})
        return True

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

    # --- named locations (LIVE since 2026-08-31, PROTOCOL.md §5.3) ---------
    #
    # A location is a pose in a MAP's coordinate frame, so it only means
    # anything alongside the map it was recorded on. Every call here resolves a
    # map first: `map` if you pass one, otherwise whichever map the running
    # navigation session has loaded. Passing `map` explicitly is what lets a UI
    # list or tidy up places saved on a map the robot is not currently using.
    #
    # The two calls that involve the ROBOT rather than just the store —
    # `goto_location`, and `save_location` in its "save where I am" form —
    # additionally require that map to be the one Nav2 has open, and are
    # refused otherwise. This is a real guard, not bookkeeping: map-frame
    # coordinates from a different map are perfectly well-formed numbers
    # pointing at a different room, and nothing downstream would catch it.
    # The robot would simply drive there.

    @staticmethod
    def _with_map(payload: dict, map: Optional[str]) -> dict:
        """Attach `map` only when the caller named one — the server falls back
        to the active session's map on absence, not on null."""
        if map is not None:
            payload["map"] = map
        return payload

    def save_location(
        self,
        name: str,
        x: Optional[float] = None,
        y: Optional[float] = None,
        theta: float = 0.0,
        map: Optional[str] = None,
    ) -> bool:
        """Save a named pose on a map.

        Two forms, and which one you get depends on whether you pass
        coordinates:

        - ``save_location("kitchen")`` records **where the robot is now**.
          Requires the robot to be localized — saving "here" while AMCL has
          not converged records a coordinate that means nothing and only fails
          much later, when someone navigates to it. Refused (raises
          :class:`~bonicos.CommandError`) unless the robot is navigating on
          the map being saved to.
        - ``save_location("kitchen", x=1.2, y=3.4)`` records **a point picked
          on the map**, which need not be the map currently loaded.

        ``theta`` is the heading to arrive facing, and only applies to the
        explicit-coordinate form (the robot's own heading is used otherwise).
        """
        payload: Dict[str, Any] = {"type": protocol.CMD_SAVE_LOCATION, "name": name}
        if x is not None and y is not None:
            payload.update({"x": x, "y": y, "theta": theta})
        self._command(self._with_map(payload, map))
        return True

    def goto_location(
        self,
        name: str,
        wait: bool = True,
        timeout: float = 60.0,
        map: Optional[str] = None,
    ) -> bool:
        """Navigate to a saved location.

        A lookup plus the normal ``nav_goal`` path, so it reports progress
        through ``nav_status`` exactly like ``go_to``, and like ``go_to`` it
        returns whether the robot got there.

        Raises :class:`~bonicos.CommandError` — before anything moves — if the
        location doesn't exist on the map, or if the robot isn't navigating on
        that map; see this section's note on why that's refused rather than
        driven.
        """
        result = self._command(
            self._with_map({"type": protocol.CMD_GOTO_LOCATION, "name": name}, map)
        )
        if not wait:
            return True
        return self.wait_for_goal(timeout, goal_id=result.get("goal_id"))

    def list_locations(self, map: Optional[str] = None) -> List[str]:
        """Names of the locations saved on a map.

        Names only — the same shape ``list_maps()`` returns, and for the same
        reason: what a caller wants is a value they can hand straight back to
        ``goto_location``. Use ``get_locations()`` for the coordinates.

        Empty (rather than an error) when there is no map to resolve, since
        "no navigation session, so no locations" is a legitimate answer to
        this question and callers reasonably iterate the result.
        """
        return [entry["name"] for entry in self.get_locations(map)]

    def get_locations(self, map: Optional[str] = None) -> List[Dict[str, Any]]:
        """Full location records: ``[{"name", "x", "y", "theta"}, ...]``,
        sorted by name. The coordinates are in the map's frame."""
        result = self._command(
            self._with_map({"type": protocol.CMD_LIST_LOCATIONS}, map)
        )
        records = []
        for entry in result.get("locations", []):
            # Tolerate the bare-name shape the handler returned while it was a
            # stub, so an older robot_app degrades to names instead of raising.
            if isinstance(entry, dict):
                records.append(entry)
            else:
                records.append({"name": entry, "x": 0.0, "y": 0.0, "theta": 0.0})
        return records

    def delete_location(self, name: str, map: Optional[str] = None) -> bool:
        self._command(
            self._with_map({"type": protocol.CMD_DELETE_LOCATION, "name": name}, map)
        )
        return True

    def delete_all_locations(self, map: Optional[str] = None) -> bool:
        """Forget every location on a map. Note locations are also dropped
        automatically when their map is deleted (``delete_map``) — a later map
        reusing the name must not inherit places from a different room."""
        self._command(
            self._with_map({"type": protocol.CMD_DELETE_ALL_LOCATIONS}, map)
        )
        return True

    # --- docking (addon only, PROTOCOL.md §5.3.1) --------------------------
    #
    # A dock is a charging station the robot reverses onto, guided by an
    # AprilTag and a rear camera, both part of the optional docking addon.
    # The SDK sends every call regardless (PROTOCOL.md §3.1); a robot without
    # the addon refuses, and that arrives as CommandError.
    #
    # Docks are map-frame poses saved per map, separate from locations, and
    # `dock()` needs a navigation session on the dock's map. Names default to
    # "default", for the usual one dock per map.

    #: The name every docking call uses when none is given.
    DEFAULT_DOCK = "default"

    #: Ack timeout for `dock`/`undock`. A robot may start its docking
    #: pipeline before accepting the goal, which can take several seconds.
    _DOCK_ACK_TIMEOUT_S = 30.0

    def save_dock(self, name: str = DEFAULT_DOCK) -> bool:
        """Record where the robot is parked right now as this map's dock.

        Drive the robot onto the dock first — exactly as it should sit when
        charging — then call this. There is no form that takes coordinates:
        a dock has to be the exact physical docked position. Saving the same
        name again overwrites it.

        Raises :class:`~bonicos.CommandError` if this robot has no docking
        addon, isn't navigating on a map, or doesn't know where it is yet.
        """
        self._command({"type": protocol.CMD_SAVE_DOCK, "name": name})
        return True

    def list_docks(self, map: Optional[str] = None) -> List[str]:
        """Names of the docks saved on a map (the current one by default).
        Empty when there are none, or no map to resolve."""
        return [entry["name"] for entry in self.get_docks(map)]

    def get_docks(self, map: Optional[str] = None) -> List[Dict[str, Any]]:
        """Full dock records: ``[{"name", "x", "y", "theta"}, ...]``, in the
        map's frame, sorted by name."""
        result = self._command(self._with_map({"type": protocol.CMD_LIST_DOCKS}, map))
        return [entry for entry in result.get("docks", []) if isinstance(entry, dict)]

    def delete_dock(self, name: str = DEFAULT_DOCK, map: Optional[str] = None) -> bool:
        """Forget a saved dock. Works on any robot, addon or not."""
        self._command(
            self._with_map({"type": protocol.CMD_DELETE_DOCK, "name": name}, map)
        )
        return True

    def dock(
        self, name: str = DEFAULT_DOCK, wait: bool = True, timeout: float = 120.0
    ) -> bool:
        """Drive to the saved dock and park on it.

        The robot navigates to a point in front of the dock, then reverses
        onto it using the dock's AprilTag. With ``wait`` it blocks until that
        finishes and returns whether the robot ended up docked;
        ``get_dock_result()`` says why when it didn't.

        Raises :class:`~bonicos.CommandError` — before anything moves — if
        this robot has no docking addon, isn't navigating on the dock's map,
        or has no dock saved under ``name`` there.
        """
        result = self._command(
            {"type": protocol.CMD_DOCK, "name": name}, timeout=self._DOCK_ACK_TIMEOUT_S
        )
        if not wait:
            return True
        return self.wait_for_dock(timeout, goal_id=result.get("goal_id"))

    def undock(self, wait: bool = True, timeout: float = 60.0) -> bool:
        """Drive straight off the dock. Returns whether that finished; raises
        :class:`~bonicos.CommandError` if this robot has no docking addon."""
        result = self._command(
            {"type": protocol.CMD_UNDOCK}, timeout=self._DOCK_ACK_TIMEOUT_S
        )
        if not wait:
            return True
        return self.wait_for_dock(timeout, goal_id=result.get("goal_id"))

    def wait_for_dock(
        self, timeout: float = 120.0, goal_id: Optional[str] = None
    ) -> bool:
        """Block until the current dock/undock attempt ends; True if it
        succeeded. The docking counterpart of ``wait_for_goal``."""
        return self._wait_for_terminal(protocol.EVENT_DOCK_STATUS, timeout, goal_id)

    def get_dock_status(self) -> str:
        """``"navigating"`` while a dock/undock runs, then ``"succeeded"``,
        ``"failed"`` or ``"canceled"``; ``"idle"`` before the first attempt."""
        event = self._latest(protocol.EVENT_DOCK_STATUS)
        return event.get("status", "idle") if event else "idle"

    def get_dock_result(self) -> Optional[Dict[str, Any]]:
        """The latest dock/undock report in full: ``{"status", "goal_id"}``,
        plus ``error`` when it never started and ``error_code`` when it failed
        partway. ``None`` before the first
        attempt."""
        event = self._latest(protocol.EVENT_DOCK_STATUS)
        return dict(event) if event else None
