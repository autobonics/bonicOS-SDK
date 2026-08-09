# bonicos — User-Facing Python API

The surface students and developers write against. Synchronous and blocking —
no `async`/`await`. Identical whether the code runs on a laptop, in an on-robot
runner, or in the browser (Pyodide, pro models). Wire details live in
[PROTOCOL.md](./PROTOCOL.md); package internals in [ARCHITECTURE.md](./ARCHITECTURE.md).

> **Marker:** methods tagged **🔌 stub** exist and are safe to call but are
> **no-ops on the robot in v1** (the ROS path is not wired yet — PROTOCOL §8).
> Code using them runs; the robot simply won't move that actuator until the
> server side lands. Everything else is fully functional in v1.

> **This is a proposal.** Method names/shape are open to redline before
> implementation — see the "Open API choices" note at the end.

---

## 1. Connect & lifecycle

```python
from bonicos import BonicBot

# Explicit host (developer laptop → robot on the LAN, or → tablet on lite models)
robot = BonicBot("192.168.1.50", robot_id="M1_001")

# On-robot runner container (localhost)
robot = BonicBot(host="127.0.0.1", robot_id="M1_001")

# Browser (Pyodide, pro models) / mDNS autodiscovery (native) — no args
robot = BonicBot()
```

| Method | Blocks? | Description |
|---|---|---|
| `BonicBot(host=None, *, robot_id=None, token=None, timeout=10.0)` | yes (connects) | Connect + handshake. Env auto-selects transport (WebSocket native / WebRTC in Pyodide). `token` defaults to `$BONICOS_TOKEN`. Raises `ConnectionError` on failure. |
| `robot.is_connected() -> bool` | no | Live connection state. |
| `robot.close()` | yes | Stop the robot, close the transport. Idempotent. |
| `robot.features -> dict[str, bool]` | no | Series feature flags from the handshake (e.g. `robot.features["navigation"]`). |
| `robot.robot_id -> str`, `robot.series -> str` | no | From the handshake. |

Supports the context-manager form, which guarantees a `stop` on exit
(recommended for every run — matches the platform "stop in a `finally`" rule):

```python
with BonicBot("192.168.1.50", robot_id="M1_001") as robot:
    robot.move_forward(duration=2)
# motors stopped, socket closed, even on exception
```

Calling a **gated** feature raises `FeatureUnavailable`; a disconnect mid-call
raises `RobotDisconnected`.

---

## 2. Motion (base movement)

High-level wrappers over the `drive` command. `duration=None` starts the motion
and returns immediately; a number blocks for that many seconds then stops.

| Method | Description |
|---|---|
| `robot.drive(linear_x=0.0, angular_z=0.0)` | Raw velocity (m/s, rad/s). Sent continuously by the SDK to satisfy the deadman while active. |
| `robot.move_forward(speed=0.3, duration=None)` | Forward at `speed` m/s. |
| `robot.move_backward(speed=0.3, duration=None)` | Backward. |
| `robot.turn_left(speed=0.5, duration=None)` | Rotate left at `speed` rad/s. |
| `robot.turn_right(speed=0.5, duration=None)` | Rotate right. |
| `robot.stop()` | Zero velocity immediately. |
| `robot.is_moving() -> bool` | From odom telemetry. |

Grouped access: `robot.motion.*` (same methods).

---

## 3. Precise motion (closed-loop)

Client-side control loops (ARCHITECTURE §4) over `drive` + odometry. **Blocking**
with a timeout; the on-robot deadman backstops a stalled loop.

| Method | Description |
|---|---|
| `robot.drive_distance(meters, speed=0.3, timeout=30.0) -> bool` | Drive straight a set distance. `True` on success. |
| `robot.rotate_angle(degrees, speed=45.0, timeout=30.0) -> bool` | Rotate in place by an angle. |
| `robot.drive_and_rotate(meters, degrees, speed=0.3, turn_speed=45.0, timeout=30.0) -> bool` | Drive then rotate. |
| `robot.draw_square(side_m, speed=0.3, turn_speed=45.0) -> bool` | Convenience pattern. |

Command queue (build a routine, then run it):

| Method | Description |
|---|---|
| `robot.enqueue(cmd_list)` | Queue precise-motion steps, e.g. `[("drive", 1.0), ("rotate", 90)]`. |
| `robot.run_queue(block=True) -> bool` | Execute the queue. |
| `robot.clear_queue()` | Flush queue and stop. |

> These signatures are frozen across the v1→on-Pi migration: when the robot side
> gains a native motion server, the loop is replaced by a single intent command
> with no change to these method signatures (PROTOCOL §4, pattern 3 → later 2).

---

## 4. Navigation, mapping & locations

Fire-and-monitor: goal methods start navigation; `wait_for_goal()` blocks on
`nav_status`. Coordinates are map-frame meters/radians.

| Method | Blocks? | Description |
|---|---|---|
| `robot.go_to(x, y, theta=0.0, wait=True, timeout=60.0) -> bool` | `wait` | Navigate to a pose (Nav2). |
| `robot.navigate_waypoints(points, wait=True) -> bool` | `wait` | `points=[(x,y,theta?), ...]`. |
| `robot.cancel_goal() -> bool` | yes | Cancel current navigation. |
| `robot.wait_for_goal(timeout=30.0) -> bool` | yes | Block until the active goal finishes. |
| `robot.get_nav_status() -> str` | no | `idle`/`navigating`/`succeeded`/`failed`/`canceled`. |
| `robot.get_distance_to_goal() -> float` | no | From `nav_status`. |
| `robot.get_plan() -> list[(x, y)]` | no | Latest planned path. Reflects whatever Nav2 last published on `/plan` — verified against the real M1 sim (2026-08-04) that this is **not** guaranteed to clear to `[]` when a goal succeeds (Nav2 just stops updating it, leaving the last path cached); use `get_nav_status()`, not an empty plan, to detect "no longer navigating." |
| `robot.set_initial_pose(x, y, theta=0.0) -> bool` | yes | Seed localization. |
| `robot.start_navigation() / stop_navigation() -> bool` | yes | Nav2 lifecycle. **🔌 stub.** |

**Nav-mode session switching** (added 2026-08-08) — brings up or tears down
the whole mapping/navigation ROS launch tree; distinct from `start_mapping`/
`stop_mapping` below, which only pause/unpause SLAM *inside* an already-
entered mapping session. Slow (multi-second launch settle), hence the longer
default timeouts:

| Method | Blocks? | Description |
|---|---|---|
| `robot.enter_mapping_mode(timeout=30.0) -> bool` | yes | Tear down any nav session, launch slam_toolbox+Nav2. |
| `robot.enter_navigation_mode(name, timeout=30.0) -> bool` | yes | Tear down any nav session, launch map_server+AMCL+Nav2 localizing on saved map `name`. `False` if the map doesn't exist or the launch fails to come up. |
| `robot.stop_nav_mode(timeout=15.0) -> bool` | yes | Tear down the current nav session → idle. Drive/sensors stay up. |
| `robot.get_nav_mode() -> dict` | yes (fresh query) | `{"mode": "idle"\|"mapping"\|"navigating", "map": str\|None, "transitioning": bool, "localized": bool}`. `localized` is freshness-checked (from `pose` staleness), not latched — a robot can be `navigating` and still `localized: False` right after entering (AMCL's seed hasn't landed) or later if it loses the pose. |

Typical mapping-then-navigating workflow: `enter_mapping_mode()` →
`start_mapping()` → drive around → `stop_mapping()` → `save_map(name)` →
`enter_navigation_mode(name)`.

**AMCL seeding is automatic** (added 2026-08-09): `enter_navigation_mode` and
`load_map` both auto-seed AMCL server-side (the last pose remembered on that
map, or its origin if new) — you don't need to call `set_initial_pose`
yourself in the common case. It can still fail on a slow host or a map with
no remembered pose; poll `get_nav_mode()["localized"]` and fall back to
`set_initial_pose` if it stays `False`.

Mapping:

| Method | Description |
|---|---|
| `robot.start_mapping() / stop_mapping() -> bool` | Pause/unpause SLAM integration within an already-entered mapping session (see `enter_mapping_mode` above — this alone doesn't launch anything). |
| `robot.save_map(name="map") -> bool` | Save the current map. |
| `robot.load_map(name) -> bool` | Swap the map a *running navigation session* localizes against (nav2 map_server's in-place `/load_map`, auto-reseeding AMCL). Only works while already in navigation mode — use `enter_navigation_mode(name)` to start one. |
| `robot.delete_map(name) -> bool` | Delete a saved map and its sidecar files. `False` if it doesn't exist or a live navigation session is currently localized against it — `stop_nav_mode()` or switch maps first. |
| `robot.list_maps() -> list[str]` | Saved map names (the server actually returns richer metadata dicts — `list_maps()` extracts just the name; verified against the real M1 sim 2026-08-04). |
| `robot.get_map() -> dict` | Latest occupancy grid (decoded from cached `map`). |
| `robot.get_costmap() -> dict` | Latest costmap (decoded from cached `costmap`), same shape as `get_map()`. |

Named locations (semantic waypoints) — **all 🔌 stub in v1**:

| Method | Description |
|---|---|
| `robot.save_location(name) -> bool` | Save current pose under a name. |
| `robot.goto_location(name, wait=True) -> bool` | Navigate to a saved location. |
| `robot.list_locations() -> list[str]` | (`[]` while stubbed.) |
| `robot.delete_location(name) -> bool` / `robot.delete_all_locations() -> bool` | Manage saved locations. |

Grouped access: `robot.nav.*`.

---

## 5. Arms, grippers & neck

Built on `servo_command` (registry camelCase joints → controller groups,
angles in **degrees** at the API boundary, converted to radians on the wire).

| Method | Description |
|---|---|
| `robot.set_servos(angles: dict, duration=1.0, wait=True, timeout=None) -> bool` | Set multiple joints, e.g. `{"leftElbow": -30, "neckYaw": 20}`. |
| `robot.move_left_arm(shoulder, elbow, wait=True, duration=1.0, timeout=None) -> bool` | Left arm shorthand. |
| `robot.move_right_arm(shoulder, elbow, wait=True, duration=1.0, timeout=None) -> bool` | Right arm shorthand. |
| `robot.set_grippers(left, right) -> bool` | Both grippers (degrees). |
| `robot.open_grippers() / close_grippers() -> bool` | Convenience. |
| `robot.set_neck(yaw) -> bool` / `robot.look_left/right/center() -> bool` | Neck yaw. |
| `robot.reset_servos() -> bool` | All 18 registry joints to neutral. |
| `robot.set_single_servo(joint, angle) -> bool` | One joint by name. **🔌 stub** (direct addressing) where no controller group covers it. |
| `robot.get_servo_angles() -> dict` | From `joint_states` telemetry, keyed by the same **registry camelCase** names (e.g. `"leftElbow"`) commands are sent with — not the raw snake_case URDF names the wire uses underneath. |

`ServoID` (§11) enumerates the exact 18 registry keys — the M1's real joint
set, not the old BLE-hardware set it was originally ported from.

> **`wait=True` means the arm actually arrived, not just that the server
> acked the command** (fixed 2026-08-04 — see `ARCHITECTURE.md` §4a for the
> test evidence). It polls `get_servo_angles()` until every commanded joint
> is within `8.6°` (`ArmController.CONVERGENCE_TOLERANCE_DEG`) of its
> target, or `timeout` elapses — default `max(duration * 3, 5.0)`, padded
> well above `duration` rather than assuming wall-clock time matches it.
> `set_servos`/`move_left_arm`/`move_right_arm` accept an explicit
> `timeout=` to override the default. A joint the server reports as
> `unknown` (a typo'd key, say) is excluded from the wait rather than
> spuriously timing out the whole call.
>
> **You never need to specify a whole arm — partial calls work correctly**
> (fixed 2026-08-04, `ARCHITECTURE.md` §4b): `move_left_arm(shoulder, elbow)`
> only names 2 of the arm's 7 joints, but the SDK automatically holds the
> other 5 at their current position so the command isn't silently ignored
> (the real controller requires a complete joint set per command — an
> internal detail you don't need to think about; just call the methods
> normally). One caveat, sim/hardware-side, not an SDK issue: on the current
> M1 sim, `leftElbow`/`rightElbow` specifically don't respond to position
> commands (0/60 in an independent stress test, `bonicOS-m1-ros/multiTestReport_stress.md`)
> — every other joint is 100% reliable.
>
> **Calling two arm/servo methods back-to-back without waiting is safe:** a
> new `servo_command` to a group immediately preempts whatever trajectory
> was still running, smoothly interpolating from the joint's current
> position to the new target — it never queues, so there's no risk of a
> stale command "catching up" later (`ARCHITECTURE.md` §4a).

Grouped access: `robot.arm.*`.

---

## 6. Head expression & display — **all 🔌 stub in v1**

Carried from the old BLE SDK; no ROS path yet, so these are safe no-ops until the
robot side lands (PROTOCOL §5.5).

| Method | Description |
|---|---|
| `robot.set_expression(mode)` | `"normal"/"happy"/"sad"/"angry"/"surprised"/"confused"` (`HeadMode` enum). |
| `robot.look(pan=None, tilt=None, speed=None)` | Head pan/tilt (prefers the head controller group where it exists). |
| `robot.set_display_text(text)` | LED-matrix text. |
| `robot.set_display_color(r, g, b)` | Matrix color. |
| `robot.set_display_animation(mode)` / `play_display()` / `pause_display()` / `clear_display()` | Matrix animation control. |
| `robot.set_display_brightness(value)` | Matrix brightness. |

Grouped access: `robot.head.*`.

---

## 7. Speech

One method; the robot decides *where* the audio is produced (Android TTS via
tablet, or the Pi's own TTS) based on model + config — the caller never picks
(PROTOCOL §5.6). **🔌 stub on pro until the ESP-relay / on-device TTS path lands;
fully live on lite** (the Flutter app serves and speaks directly).

| Method | Description |
|---|---|
| `robot.speak(text, voice=None) -> bool` | Say `text`. Blocks until accepted. |

---

## 8. Sensors & telemetry

Telemetry is pushed continuously and cached; reads are **non-blocking** and
return the latest value. Use `wait_for_update()` to pace loops to the real sensor
rate.

| Method | Description |
|---|---|
| `robot.get_position() -> dict` | `{x, y, theta}` (map frame). |
| `robot.get_x() / get_y() / get_heading() -> float` | Individual pose fields (heading in degrees). |
| `robot.get_battery() -> float` | State of charge (%). |
| `robot.get_imu() -> dict` | `{ax, ay, az, gx, gy, gz}`. |
| `robot.get_distance_traveled(start=None) -> float` | Odometry-derived. |
| `robot.wait_for_update(timeout=1.0) -> bool` | Block until the next telemetry frame. |
| `robot.wait_for_data(timeout=5.0) -> bool` | Block until first telemetry arrives after connect. |
| `robot.subscribe(events)` | Narrow the telemetry stream (e.g. `["pose", "battery"]`). |

**Recommended loop pattern** (from `bonic-architecture.md` §5 — never spins,
self-paces to the sensor rate, works identically in the browser):

```python
while robot.wait_for_update():
    if robot.get_battery() < 15:
        robot.speak("Low battery, returning to base")
        robot.goto_location("charger")
        break
```

Grouped access: `robot.sensors.*`.

---

## 9. Camera

Video is WebRTC under the hood on every transport, but you never have to
think about that — call `get_camera_frame()`/`get_frame()` and the link
comes up transparently on first use. Frames are **BGR `numpy` arrays**
(OpenCV's native layout), same shape on every transport. A multi-camera
robot (e.g. the M1's face and docking cameras) exposes each by name.

| Method | Description |
|---|---|
| `robot.list_cameras() -> list[str]` | Camera names from the connect handshake. Available on any transport (informational) — frames still need a video path. |
| `robot.get_camera_frame(camera=None) -> ndarray \| None` | Latest BGR frame for `camera` (default: the first), or `None` if none has arrived yet. Starts the stream on first call. |
| `robot.camera.get_frames() -> dict[str, ndarray \| None]` | Latest frame for every camera, keyed by name. |
| `robot.camera.start(cameras=None)` | Bring the stream up now instead of lazily on first `get_frame()`. Blocks until the link is established (or raises `CameraUnavailable` on timeout/no video path). |
| `robot.camera.stop()` | Tear down the video path (idempotent). Commands/telemetry are unaffected. |

If the transport genuinely has no video path (the offline mock) or the
browser host didn't wire one up, camera calls raise `CameraUnavailable`
rather than silently returning `None` forever.

**Native (WebSocket) vs browser (Pyodide/WebRTC):** on native, the SDK opens
its own `aiortc` peer to the robot on first use — `pip install
bonicos[camera]` for the extra deps (`aiortc`, `numpy`). In the browser, the
host already owns the WebRTC peer connection and decodes frames into a
`SharedArrayBuffer` the SDK reads directly (numpy must be preloaded as a
Pyodide package — no `aiortc`/`av` needed there, the host does the
decoding); see `ARCHITECTURE.md` §3.2 and `webrtc.py`'s module docstring for
the exact byte layout. Either way the student-facing call is identical.

Grouped access: `robot.camera.*`.

---

## 10. System

| Method | Description |
|---|---|
| `robot.health() -> dict` | CPU / RAM / temperature / container status. |
| `robot.restart_base_session(timeout=120.0) -> bool` | Recover a wedged robot: restart the ROS stack *underneath* mapping/navigation (drive, controllers, EKF, sensors, TF) — nav session down, base down, base up, nav session back. **🔒 gated on `session_control`.** Refused while the robot is moving or running a nav goal — cancel/stop first. Slow (cold-start Gazebo alone is ~25s); the long default timeout reflects that, and a WebRTC video peer will drop partway through since the restart takes the camera topics with it. |
| `robot.get_session_status() -> dict` | Fresh, synchronous `{"base": {...}, "nav": {...}, "health": {...}}` — the full picture behind `system.get_base_session()`/`get_session_health()` in one round trip, without waiting for a push. |
| `robot.reconfig_wifi(ssid, password) -> bool` | Apply Wi-Fi credentials. |
| `robot.trigger_update() -> bool` | Pull + restart the robot app. |
| `robot.ask_llm(prompt, model=None) -> str` | On-device LLM (S/M series). **Display only** — output is never executed as a command. Blocks and returns the full text (tokens stream internally). |

Grouped-only (`robot.system.*`, not flattened onto `robot.*` — mirrors
`get_plan()`/`get_costmap()`):

| Method | Description |
|---|---|
| `robot.system.get_base_session() -> dict \| None` | Latest cached `base_session` telemetry: `{"running", "owned", "transitioning", "error"}`. `None` before the first frame arrives. |
| `robot.system.get_session_health() -> dict \| None` | Latest cached `session_health` telemetry: `{"ok", "base", "nav", "issues"}` — `issues` names the mechanism (e.g. `"amcl_not_running"`, `"pose_stale:23s"`), not just a boolean. Pushed only on change, so may still be `None` right after connecting even on a healthy robot; use `get_session_status()` for a guaranteed-fresh read. |

Grouped access: `robot.system.*`.

---

## 11. Enums & exceptions

```python
from bonicos import HeadMode, ServoID          # enums (trimmed to core)
from bonicos import (
    RobotError,            # base
    ConnectionError,       # connect/handshake failed
    CommandError,          # server returned `error`
    FeatureUnavailable,    # gated-off feature for this series
    RobotDisconnected,     # link dropped mid-call
)
```

`FeatureUnavailable` and `RobotDisconnected` surface as **real Python
exceptions** inside user code (platform requirement) so student programs can
`try/except` them.

---

## 12. Worked examples

**Square patrol with obstacle awareness**

```python
from bonicos import BonicBot

with BonicBot("192.168.1.50", robot_id="M1_001") as robot:
    robot.wait_for_data()
    for _ in range(4):
        robot.drive_distance(1.0)
        robot.rotate_angle(90)
    robot.speak("Patrol complete")
```

**Navigate to a saved place, then gesture** (locations/head are 🔌 stub — runs,
but only navigation moves the robot in v1)

```python
with BonicBot(robot_id="M1_001") as robot:      # autodiscovery
    if robot.features["navigation"]:
        robot.goto_location("kitchen")          # blocks until arrival
    robot.set_expression("happy")               # no-op in v1
    robot.move_right_arm(shoulder=90, elbow=-30)
```

**Browser (Pyodide) — identical code, pro models only**

```python
robot = BonicBot()                 # binds the preloaded transport
while robot.wait_for_update():
    robot.drive(linear_x=0.2)
    if robot.get_distance_traveled() > 2.0:
        robot.stop(); break
```

---

## Open API choices (please redline before implementation)

1. **Class name / shape.** Proposed: `BonicBot` primary (brand-familiar), with an
   optional `Robot` alias for the architecture doc's examples, flat convenience
   methods **plus** grouped controllers (`robot.motion`, `robot.nav`, …). Pure-flat
   or pure-namespaced are alternatives.
2. **Angle units.** Proposed: **degrees** at the API boundary (matches both old
   SDKs' student ergonomics), radians on the wire. Confirm.
3. **`duration`-blocks-vs-returns** on motion helpers. Proposed: `None` =
   fire-and-return, number = block-then-stop (matches `bonicbot-bridge`).
4. **`wait=True` default** on nav/arm methods (block by default). Confirm vs
   fire-and-forget default.
5. **Naming migration.** A few method names differ from the old SDKs
   (`go_to` vs `goto`, `set_expression` vs `control_head(mode=)`). Since there's
   no backward-compat requirement, proposing the cleaner names — flag any you
   want preserved.
