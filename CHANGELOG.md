# Changelog

All notable changes to `bonicos`. This project follows
[Semantic Versioning](https://semver.org/); while on `0.x`, breaking changes
bump the minor version.

## [Unreleased]

## [0.7.0] — 2026-09-06

The head is real. `bonicOS-robot-app` grew the face-matrix path, and
`bonicOS-firmware` flipped the elbow's sign — this release catches the SDK up
to both.

### Added

- **The head expression and LED matrix API is live on A series.** API.md §6 was
  a documented no-op ("all 🔌 stub in v1"); `bonicOS-robot-app` now packs the
  `CMD_MATRIX_ACTION` body and publishes it to the ros2_control plugin, which
  forwards it to the ESP. No SDK method changed name or signature — they were
  always sending real commands — but they now do something. Needs the base
  stack up, since the plugin owns `/dev/esp`.
- `look()` takes a keyword-only `duration` (seconds). `speed` is still accepted
  positionally so the published signature keeps working, but the robot ignores
  it: ros2_control position groups take a time, not a rate.
- **`DisplayAnimation`** (exported from `bonicos`) names the fifteen animations
  the matrix can play — `rainbow_wave`, `fire`, `plasma`, `matrix_rain`,
  `battery`, the four faces, and the rest. `set_display_animation()` takes a
  member, a bare string, or a raw firmware index for anything the enum has not
  named yet. Values mirror `robot_app`'s `ANIMATION_MODES`, which mirrors the
  firmware enum; a test pins them together.
- **`protocol.ELBOW_RANGE_DEG`**, and the note explaining why it exists.

### Fixed

- **`look(pan=30)` asked the robot for 30 radians.** `head.look()` forwarded its
  arguments to the wire unconverted while API.md §5 promises degrees at the API
  boundary — `arm.set_neck()` converts, `head.look()` did not. Harmless while
  the server was a stub; a 1718° neck command the moment it wasn't. Now
  converted here, like every other angle in the SDK.
- **`look()` reported success for motion that never happened.** Asking for
  `tilt` alone on an A2 — which fits neck yaw but no neck pitch — returned
  `True`. It now returns `False` when the robot drove none of the axes asked
  for, and `True` when at least one moved.
- **A refused display command threw away the robot's reason.** "No LED matrix
  on this series" and "the base stack is down" come back as `ok: False` plus an
  `error` inside a NORMAL ack, not a protocol-level error, so nothing raised
  and the caller was left with a bare `False`. The sentence is now re-raised as
  a `UserWarning` — when the panel stays dark, it is the whole diagnosis.

### Changed

- `set_expression()` raises a `UserWarning` when the robot substitutes an
  expression it has no face for. Firmware has no `surprised` or `confused`
  face; those show a heart and a colour effect. The call still succeeds — the
  warning is so nobody builds material around a face the robot cannot make.
- **Elbow angles are positive now.** bonicOS-firmware `678dc38` inverted the
  elbow servo range: it ran −50..0 (A), −90..0 (S), −110..0 (M) and now runs
  0..50 / 0..90 / 0..110, with 0 still the arm straight. The motion is
  unchanged; the sign that describes it is not. Docs, examples and tests
  throughout the SDK now use positive elbow angles.

  **This is a silent break for anything holding a stored negative elbow
  angle** — a saved sequence, a worksheet, an old snippet. It does not error;
  it clamps at 0, so the arm simply never bends and
  `move_left_arm(shoulder=60, elbow=-30)` now means "hold the arm straight".
  The SDK deliberately does not clamp or rewrite user angles (PROTOCOL.md
  §3.1 — the ESP and the URDF enforce their own limits), so this cannot be
  fixed for you at the SDK boundary. Update the stored values.

  **Known gap, outside this repo: the ROS lane has not caught up.** Both URDFs
  still declare the old range (`bonicbot-a2-ros` `body.xacro`
  `lower="-0.873" upper="0"`, `bonicOS-m1-ros` `lower="-1.9199" upper="0.0"`).
  ros2_control clamps to the URDF, so on a Pro robot a positive elbow is
  clamped to 0 by ROS and a negative one is clamped to 0 by the ESP — the
  elbow does not move through the ROS path either way until those two files
  are re-signed.

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
