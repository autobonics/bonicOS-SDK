"""Navigation, mapping, and named locations (API.md §4).

Goal methods are fire-and-monitor: they kick off Nav2 and (with
wait=True, the default) block on `wait_for_goal()`. Navigation needs a lidar
and an on-board computer, so this whole file is **Pro only** — on a Lite
robot every call here raises `CommandError` with the robot's own explanation.
There is nothing to check first: send it and catch the error (shown below).

Nav-mode session switching (added 2026-08-08): `enter_mapping_mode()` /
`enter_navigation_mode(name)` / `stop_nav_mode()` bring up or tear down the
WHOLE ROS launch tree (slam_toolbox+Nav2, or map_server+AMCL+Nav2) — exactly
one of mapping/navigation can be up at a time, and neither `go_to`/`nav_goal`
nor `load_map` do anything useful until one is entered. This is distinct
from `start_mapping()`/`stop_mapping()` below, which only pause/unpause SLAM
integration *inside* an already-entered mapping session. These session
switches are slow (multi-second ROS launch settle) — expect `enter_*` calls
to take several seconds.

Named locations (save_location/goto_location/...) are all 🔌 stub in v1 —
they run without error but don't yet do anything server-side.

Note: `get_plan()`/`get_costmap()` are only exposed on the grouped
controller (`robot.nav.get_plan()`), not as flat `robot.*` methods — used
that way below.

This moves the robot when navigation is available. Set HOST below before running.
"""

from bonicos import BonicBot, CommandError

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim


def main() -> None:
    with BonicBot(HOST) as robot:
        robot.wait_for_data()

        # Probe once: on a robot without navigation this raises immediately
        # and the message comes straight from the robot.
        try:
            robot.go_to(0.0, 0.0, wait=False)
            has_nav = True
        except CommandError as exc:
            print(f"No navigation on this robot: {exc}")
            has_nav = False

        if has_nav:
            input(
                "This will physically move the robot to a goal pose. "
                "Press Enter to continue (Ctrl+C to abort)..."
            )

            print("Entering mapping mode (launches slam_toolbox+Nav2)...")
            ok = robot.enter_mapping_mode()
            print(f"enter_mapping_mode() -> {ok}, nav_mode={robot.get_nav_mode()}")

            print("Mapping a bit of the space...")
            robot.start_mapping()
            ok = robot.go_to(1.0, 0.0, theta=0.0, timeout=30.0)
            print(f"go_to(1.0, 0.0) -> {ok}, status={robot.get_nav_status()}")
            robot.stop_mapping()
            robot.save_map("example_map")
            print("Saved maps:", robot.list_maps())

            print("Switching to navigation mode on the saved map...")
            ok = robot.enter_navigation_mode("example_map")
            mode = robot.get_nav_mode()
            print(f"enter_navigation_mode('example_map') -> {ok}, nav_mode={mode}")

            # The server auto-seeds AMCL on entering navigation (the last pose
            # remembered on this map, or its origin if new) — no manual
            # set_initial_pose call needed in the common case. It can still
            # miss its deadline on a slow host, though, so check `localized`
            # and fall back to a manual placement if it's still False.
            if not mode["localized"]:
                print("auto-seed didn't land yet — placing the robot by hand")
                robot.set_initial_pose(0.0, 0.0, 0.0)

            occ = robot.get_map()
            print("get_map() keys:", sorted(occ.keys()) if occ else occ)

            # load_map only makes sense here on — swapping to a DIFFERENT
            # saved map while already navigating, without a full session
            # relaunch (and auto-reseeding AMCL for the swap). Re-loading the
            # same map is a no-op demo of the call.
            loaded = robot.load_map("example_map")
            print("load_map('example_map') ->", loaded)

            # navigate_waypoints blocks until the whole route finishes.
            ok = robot.navigate_waypoints([(0.5, 0.0), (0.5, 0.5, 90)], wait=True)
            print(f"navigate_waypoints(...) -> {ok}")

            # Fire-and-monitor form: start, poll, cancel.
            robot.go_to(2.0, 2.0, wait=False)
            print("nav_status right after issuing the goal:", robot.get_nav_status())
            print("distance_to_goal:", robot.get_distance_to_goal())
            print("planned path points:", len(robot.nav.get_plan()))
            robot.cancel_goal()
            robot.wait_for_goal(timeout=5.0)
            print("nav_status after cancel:", robot.get_nav_status())

            print("Leaving nav mode (idle) — base drive/sensors stay up...")
            robot.stop_nav_mode()
            print("final nav_mode:", robot.get_nav_mode())

            # Now safe to delete — no live session is localized against it
            # (refused with False while one is).
            print("delete_map('example_map') ->", robot.delete_map("example_map"))

        # Named locations — safe to call regardless of series; all stub in v1.
        robot.save_location("home")
        print("list_locations() (stub, expect []):", robot.list_locations())


if __name__ == "__main__":
    main()
