# Changelog

All notable changes to `bonicos`. This project follows
[Semantic Versioning](https://semver.org/); while on `0.x`, breaking changes
bump the minor version.

## [0.10.1] — 2026-09-22

### Fixed

- **`bonicos.ai` in the browser simulator says to use a robot.** 0.10.0's
  detectors (`detect_objects`, `detect_faces`, `detect_markers`,
  `detect_gestures`) imported numpy before checking where they were running.
  Pyodide has not loaded numpy or OpenCV, so in the simulator they died with
  `ModuleNotFoundError: No module named 'numpy'` instead of raising
  `AIUnavailable` — "AI models run on a robot, not in the simulator".
  `ai.load` was unaffected. Found by running the 0.10.0 wheel in the real
  Pyodide runtime; 0.10.0 was never uploaded to PyPI or deployed.

## [0.10.0] — 2026-09-22

### Added

- **`bonicos.ai` — computer vision on the robot.** `from bonicos import ai`
  gives frame-in functions that are independent of any robot connection
  (API.md §9.1):
  - `ai.load(name).predict(frame)` runs a model trained in the Train tab — a
    `bonic-head-v1` head on the robot's bundled MobileNetV2 backbone. Every
    head is validated before it runs: checksum, byte length against its
    layers, size caps, the backbone *build* it was trained against, and the
    preprocessing it asks for. A head that fails is refused with
    `InvalidModel` rather than allowed to predict confident nonsense.
  - Built-ins that need no training: `detect_objects` (YOLOX-nano, the 80
    COCO classes), `detect_faces` (YuNet), `detect_markers` (ArUco),
    `detect_gestures` (MediaPipe).
  - New exceptions `AIUnavailable`, `ModelNotFound`, `InvalidModel`.

  The models come from the robot image (`$BONICOS_MODELS_DIR`), and trained
  models from the program's own run (`$BONICOS_AI_MODELS`,
  `$BONICOS_AI_HEADS_DIR`) — both set by robot_app. `import bonicos.ai` loads
  no OpenCV or MediaPipe, so it is safe in the browser simulator, where every
  call raises `AIUnavailable` with a sentence saying to use a robot.

  Verified on an A Pro (RPi 4) under the runner's CPU and memory limits: a
  head built from real photos classified each correctly, including mirrored
  and resized copies; peak memory with every model loaded was 347 MB of the
  512 MB allowed.

  New `[ai]` extra (`numpy`, `opencv-python-headless`) for use off the robot.
  mediapipe is deliberately not in it: the robot pins 0.10.18, because 1.x
  crashes on a Raspberry Pi 4.

  Minor rather than patch: a new public module and new exceptions.

## [0.9.0] — 2026-09-15

### Added

- **Unix-socket transport.** `BonicBot()` now resolves `$BONICOS_UDS` before
  `$BONICOS_HOST`, and `WebSocketTransport` takes a `uds=` path that switches
  the same wire protocol onto `websockets.sync.client.unix_connect`. Nothing
  about the protocol changes — only the connect call.

  This is what makes the on-robot `run_code` runner work in the container.
  Its sandbox runs with no network namespace at all (`bwrap --unshare-net`),
  so `127.0.0.1:8080` has no loopback to resolve on; a unix socket is a
  filesystem object, so one bind-mounted socket reaches robot_app while
  everything else — the LAN, the internet, and bonic-host's API on
  `127.0.0.1:8090`, which owns wifi, power-off and image installs — stays
  unreachable. Verified end-to-end on an A2 (aarch64, privileged container):
  `unix_connect` → uvicorn UDS → FastAPI `/ws`, inside `bwrap --unshare-net`,
  with `?robotId=` preserved.

  Minor rather than patch because `WebSocketTransport.__init__` grew a
  keyword — any test double standing in for it needs `uds=None` in its
  signature.

  An explicit `BonicBot(host=...)` still wins over an ambient `$BONICOS_UDS`:
  naming a host is a deliberate override. Note it will not resolve from
  inside the runner sandbox, which is exactly the mistake the env vars exist
  to prevent.

## [0.8.1] — 2026-09-15

### Fixed

- **`look_right()` turns right.** It sent `neckYaw=-45`, and `look_left()`
  sent `+45` — backwards, verified against hardware. Both flipped: right is
  now positive, left is negative. If your code called `set_neck()` directly
  with an angle copied from watching which way `look_left`/`look_right`
  turned, that angle now points the opposite way — the shorthands did, and
  still do, but in the other direction.

  **Released as a distinct version rather than folded into 0.8.0**, even
  though nothing had shipped: the 0.8.0 wheel had already been built and
  fetched into a running browser tab (`bonicAI-frontend/public/pyodide/`,
  cached `immutable` — see `next.config.ts`, which relies on exactly this:
  the wheel's version is the only thing that busts that cache). Rebuilding
  `bonicos-0.8.0-py3-none-any.whl` in place with different bytes, same
  filename, is the one thing that comment says not to do — a tab that had
  already loaded it would keep the old, still-backwards behavior forever, no
  matter what shipped afterward under that name. 0.8.0 is left as originally
  built; this fix needed a URL that hadn't been cached yet.

## [0.8.0] — 2026-09-15

### Added

- **`system.update_status()` and `system.get_update_status()`**, and the
  `update_progress` telemetry event behind them. An install triggered by the
  robot's host is minutes long, with no reply until it finishes, so there was
  previously no way to find out what happened. The robot now pushes the phase
  and the pull percentage while it downloads, and — because the install
  replaces the process doing the pushing — replays the outcome (`last_update`:
  installed, or rolled back, and why) to whatever connects afterwards.

- **`system.shutdown()`** — power the robot off, latch and all. `robot_app`
  has answered `shutdown` since the power-manager work landed and the SDK had
  no way to send it, so a program could restart the ROS stack but not the
  machine under it.

### Changed

- **`health()` is documented as the shape it actually returns** —
  `cpu_percent` / `ram_percent` / `disk_percent` / `temps`, not `cpu` / `ram`
  / `temp`, and with no container status: `robot_app` stopped updating itself,
  so the host owns that now and `update_status()` answers it. No code change
  — `health()` always returned the robot's dict verbatim — but `API.md`,
  `PROTOCOL.md` and the simulator all described the old one, and the
  simulator *served* it, which is the version that could be tested against and
  still break on hardware.

- **`protocol.ELBOW_RANGE_DEG`'s A-series max is 45, not 50.** `bonicbot-a2-ros`
  `6098b2d` had already re-signed its URDF to `0..0.785` rad (0..45°) as part
  of the sign flip; rather than widen the URDF to the SDK's old 50°, the
  firmware's A-series limit (`servo_config.cpp`, `SERVO_MAX_LIMITS[4]/[11]`)
  was brought down to 45 to match, and the SDK follows. One number now, not
  two that disagreed. **Breaking**: `move_left_arm(shoulder=60, elbow=50)` on
  an A series, which used to reach 50°, now clamps at the URDF's 45° through
  the ROS path (the SDK itself does not clamp) — update any saved sequence
  or worksheet holding an A-series elbow angle above 45.

### Fixed

- **The simulator no longer answers `update_status` with a bare `{"ok": True}`.**
  It reports `state: "unavailable"` — the same thing a real robot says when no
  bonic-host answers — so `update_status()["state"]` is readable everywhere
  instead of raising `KeyError` only under simulation. `shutdown()` likewise
  returns `False` there rather than acking a poweroff that cannot happen.

- **`MockTransport` stopped serving a `features` map in `auth_result`.**
  Capability gating was removed in 0.5.0 and the robot has not sent one since;
  the mock kept teaching tests a shape that no longer exists.

- **The elbow sign-flip caveat now matches the two ROS workspaces.** 0.7.0 said
  both URDFs still declared the old negative range; `bonicbot-a2-ros` `6098b2d`
  has since re-signed `body.xacro` to `0..0.785` and negated the axis, so on an
  A2 a positive elbow bends the arm and the warning had inverted into a lie —
  it told A2 users the elbow would not move at all. `bonicOS-m1-ros` is still
  on `-1.9199..0.0`, so the warning stands there and now says so specifically.

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
