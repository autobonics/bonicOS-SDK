# Changelog

All notable changes to `bonicos`. This project follows
[Semantic Versioning](https://semver.org/); while on `0.x`, breaking changes
bump the minor version.

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
