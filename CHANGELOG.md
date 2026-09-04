# Changelog

All notable changes to `bonicos`. This project follows
[Semantic Versioning](https://semver.org/); while on `0.x`, breaking changes
bump the minor version.

## [0.6.0] — 2026-09-04

Catch-up with `bonicOS-robot-app`'s `dev-ma-01-unify-robot-series` merge, which
made the A2 a first-class series alongside the M1.

### Fixed

- **Arm, neck and gripper commands returned `False` on A2 while working.**
  `move_left_arm()`, `set_neck()` and `reset_servos()` expanded each command to
  the full 18-actuator M1 joint group, then waited for every joint in it to
  reach its target. An A2 fits 7 of those 18, and a joint that doesn't exist
  never reports a position, so the call timed out after 5s and reported failure
  even though the arm had moved correctly. Commands and convergence waits are
  now scoped to the joints the robot actually reports in `joint_states` — the
  one honest answer to "what does this robot have", and robust to per-robot
  fitment rather than just to the three series.
- **`open_grippers()` / `close_grippers()` always returned `False`.** They
  commanded ±90°, a range that exists on no robot: the gripper travels −45°..60°
  on A/S and −60°..60° on M. The URDF clamped the command, then `wait=True` sat
  waiting for an angle the joint physically cannot reach. Now +60°/−45°, valid
  on every series. (Cross-checked against `bonicOS-firmware`'s actuator
  registry and both series' URDFs.)
- **The SDK no longer sends `0.0` for a joint it has never seen.** Filling a
  group from the joint table meant unrequested joints were commanded to zero,
  walking straight past the robot's own guard against exactly that — it refuses
  to guess `0.0` because an unrequested joint snapping to zero is a hazard on
  hardware.
- `servo_command`'s `unsupported` reply field (a real registry joint this robot
  doesn't fit, as distinct from `unknown`, which isn't a registry joint at all)
  is now honoured when deciding what to wait for.
- **Simulator: every navigation goal that needed a turn was unreachable.**
  `SimTransport`'s pure pursuit measured its lookahead from a fixed path
  vertex, and the planner emits as few as two points for a clear run — so the
  carrot never moved and the robot orbited it at its ~0.24 m turning radius
  until the caller's timeout. Measured: a goal at (1.0, 0.8) left the robot
  circling near the origin indefinitely, while (1.0, 0.0) succeeded, because
  with no heading error the orbit degenerates to a straight line. The lookahead
  is now taken from the robot's projection onto the path, which also makes the
  result independent of how often the sim is ticked — `wait_for_goal` spins
  without sleeping, so the controller had been sensitive to that. Affected
  `go_to`, `navigate_waypoints` and `goto_location` alike, in the browser
  simulator students use.

### Added

- **Named locations are real.** `save_location` / `goto_location` /
  `list_locations` / `delete_location` / `delete_all_locations` were stubs and
  are now live on the robot. Locations are **map-scoped**, so all of them take
  an optional `map=`; `save_location` gained `x`/`y`/`theta` for saving a point
  picked on a map rather than the robot's current pose. New
  `nav.get_locations()` returns the full `{name, x, y, theta}` records.
  `SimTransport` implements these too, rather than acking and forgetting.
- `sensors.get_scan()`, `get_scan_points()` and `set_scan_enabled()` for the
  new on-demand `scan` telemetry event (map-frame, downsampled; off by default
  because 10 Hz of ~1000 ranges isn't worth carrying unwatched).
- `camera.pause()` / `camera.resume()` — stop the robot encoding video nobody
  is watching (~32% of a core on a real A2) without tearing down the track.
- `system.start_base_session()` / `stop_base_session()`. These matter because
  robot_app no longer autostarts the base stack on real hardware: a robot that
  booted with no stack previously had no way back short of SSH.

### Changed

- `nav.list_locations()` returns names, extracted from the record dicts the
  server actually sends (same treatment `list_maps()` already had). It
  previously passed the raw list through, which only looked correct because the
  stub always returned `[]`.
- `nav.goto_location(..., wait=False)` returns the ack's `ok` instead of an
  unconditional `True`, so a refused location (missing, or on the wrong map)
  is not reported as a started goal.
- Documentation: `restart_base_session` is no longer described as gated on a
  `session_control` feature flag — capability gating was removed from robot_app
  and that flag no longer exists.

## [0.2.0] — unreleased

### Removed — breaking

- **`FeatureUnavailable` is gone.** A robot that cannot perform a command now
  replies with an ordinary error, which the SDK raises as `CommandError` with
  the robot's own explanation. Replace `except FeatureUnavailable` with
  `except CommandError`.

  ```python
  # before
  try:
      robot.go_to(1.0, 2.0)
  except FeatureUnavailable:
      ...

  # after
  try:
      robot.go_to(1.0, 2.0)
  except CommandError as exc:
      print(exc.reason)   # e.g. "this robot has no navigation (no lidar)"
  ```

- **`robot.features` is gone.** The handshake carries identity only
  (`robot_id`, `series`, `cameras`) and no longer advertises capability. There
  is no `robot.model`, `robot.variant`, `robot.is_pro` or `robot.joints`
  either. What each model supports is documented in
  [API.md](./API.md) — every section carries an **On Lite** line. To discover
  which actuators a robot actually has at runtime, use `get_servo_angles()`,
  which reports exactly the fitted set.

### Changed

- **`robot_id` is now optional everywhere.** `BonicBot("192.168.1.50")` is a
  complete call — `host` alone identifies a robot. Passing `robot_id` still
  works and does two things: it acts as a wrong-robot guard at handshake time
  (the connection is refused on a mismatch, which catches a stale IP after a
  DHCP change), and it narrows mDNS discovery when no `host` is given. Code
  that previously *had* to pass `robot_id` keeps working unchanged.
- Connecting no longer raises `ConnectionError("robot_id is required")`.

### Added

- **`BonicBot.simulated(joints=[...])`** — simulate a robot built with fewer
  than the full 18 actuators, since servo count is a per-robot build option:

  ```python
  robot = BonicBot.simulated(joints=["leftElbow", "neckYaw"])
  robot.get_servo_angles().keys()   # only those two
  ```

  Joints that are valid but not fitted are reported `unknown`, so
  `set_servos(wait=True)` fails fast instead of blocking until timeout.

### Fixed

- A server that closes the connection during the handshake (for example on a
  `robot_id` mismatch) now raises an error naming the actual cause, instead of
  misreporting it as `"timed out waiting for auth_result"` after the full
  timeout elapsed.
- Precise motion and the drive keepalive no longer fail on single-threaded
  hosts (Pyodide's default WASM build). The precise-motion queue runs inline
  there, so `block=False` behaves synchronously on those hosts.

### Known limitation

Naming a joint the robot does not physically have is the one failure with no
error: `set_servos(wait=True)` waits for a joint that will never move and times
out. Use `get_servo_angles()` to see the fitted set.

## [0.1.1] — 2026-08-10

- Python requirement lowered to **3.10+** to match Ubuntu 22.04 (ROS Humble's
  target OS), which every bare-metal robot runs.
- `websockets` moved from the optional `native` extra to a base dependency, so
  `pip install bonicos` produces a package that can actually connect.
- Documentation clarifications around simulation navigation and the arm
  movement API.

## [0.1.0] — 2026-08-10

Initial release: `BonicBot` client with motion, precise motion, navigation and
mapping, arms/grippers/neck, head expression and display, speech, sensors and
telemetry, camera, and system controllers; WebSocket and simulation transports;
optional `camera` and `discovery` extras.
