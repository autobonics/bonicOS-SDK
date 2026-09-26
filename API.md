# bonicos — User-Facing Python API

The complete method reference. Synchronous and blocking — no `async`/`await`.
Identical whether your code runs on your own machine or on the robot itself.
Wire details live in [PROTOCOL.md](./PROTOCOL.md).

> **Marker:** methods tagged **🔌 stub** exist and are safe to call but are
> **no-ops on current robot firmware** — the robot accepts the command and
> does nothing. Your code runs; that actuator just won't move until the robot
> side lands. Everything else is fully functional.

> **Lite robots.** Each section below carries an **On Lite** line. Lite models
> (`s1-lite`, `a2-lite`) have no on-board computer, so navigation, mapping and
> session control are **absent, not stubbed** — they raise `CommandError`
> rather than quietly doing nothing, because no later firmware makes them work.
> Everything else behaves as documented.

> **This page is the capability reference.** The SDK does not check what your
> robot supports and the robot does not advertise it — there is no
> `robot.features`. You send a command; if the robot cannot do it, it replies
> with an error explaining why. The **On Lite** lines and 🔌 markers here are
> how you know in advance. See [PROTOCOL.md](./PROTOCOL.md) §3.1.

> **Errors.** Every method that sends a command raises `CommandError` if the
> robot refuses it or fails, with the robot's reason as the message. `-> bool`
> methods return `True` on success. Methods that wait for an outcome — `go_to`,
> `navigate_waypoints`, `goto_location`, `dock`, `undock`, `wait_for_*` —
> return `False` when the goal was accepted but not reached. See §11.

---

## 1. Connect & lifecycle

```python
from bonicos import BonicBot

# Explicit host (developer laptop → robot on the LAN, or → tablet on lite models)
robot = BonicBot("192.168.1.50")

# Optional robot_id: a wrong-robot guard on a LAN with several robots, or
# the filter for mDNS discovery when host isn't known either.
robot = BonicBot("192.168.1.50", robot_id="M1_001")

# Everywhere the environment already knows which robot this is:
#   the on-robot runner ($BONICOS_HOST/$BONICOS_ROBOT_ID),
#   or mDNS autodiscovery.
robot = BonicBot()
```

**A bare `BonicBot()` is a working program wherever the environment already
knows which robot you mean** — which is what lets the same file run unchanged
on your laptop and on the robot itself. It looks for a target in this order:

| Order | Source | Typical use |
|---|---|---|
| 1 | the `host` / `robot_id` arguments | you, naming a robot from your own machine |
| 2 | `$BONICOS_HOST` / `$BONICOS_ROBOT_ID` | set for you when your code runs on the robot |
| 3 | mDNS autodiscovery | `pip install bonicos[discovery]` |

`robot_id` is optional at every step — `host` alone is enough to connect.

| Method | Blocks? | Description |
|---|---|---|
| `BonicBot(host=None, *, robot_id=None, token=None, timeout=10.0)` | yes (connects) | Connect + handshake, resolving the target per the table above. `robot_id`, if given, must match the robot's own id or the connection is refused. `token` defaults to `$BONICOS_TOKEN`. Raises `ConnectionError` on failure, naming every way to supply what was missing. |
| `robot.is_connected() -> bool` | no | Live connection state. |
| `robot.close()` | yes | Stop the robot, close the transport. Idempotent. |
| `robot.robot_id -> str`, `robot.series -> str` | no | From the handshake. Identity, for display. |
| `robot.cameras -> list[str]` | no | Camera names this robot streams, for `get_camera_frame(name)`. Empty means none. |

> **There is no `robot.features`, `robot.model`, `robot.variant`, `robot.is_pro`
> or `robot.joints`.** Capability is documented here, not advertised at runtime
> — the **On Lite** line in each section is how you know in advance. To discover
> which actuators a robot actually has, use `get_servo_angles()` (§5), which
> reports exactly the fitted set.

### Trying it without hardware

`BonicBot.simulated()` connects to a fake robot instead — no network, no
physical robot required:

```python
robot = BonicBot.simulated()
robot.move_forward(speed=0.3, duration=2)
print(robot.get_position())   # a real integrated pose, not a stub value
```

Driving, arms, and telemetry all behave for real — `get_servo_angles()`
converges the same way it would against hardware. Navigation and mapping
calls ack and do nothing (no Nav2/SLAM is simulated), the same as their
**🔌 stub** counterparts on real firmware. There's no separate API to learn:
every method on this page works identically against `BonicBot.simulated()`
and against a real robot.

**Simulating a robot with fewer actuators.** Servo count is a build option, so
the simulator can imitate a robot that shipped with ten joints instead of
eighteen:

```python
robot = BonicBot.simulated(joints=["leftElbow", "rightElbow", "neckYaw"])
print(robot.get_servo_angles().keys())   # only the three
```

> One consequence of "no Nav2 simulated" worth knowing: a goal never
> completes, so `go_to()`/`goto_location()`/`navigate_waypoints()` (`wait=True`
> by default) block for their full `timeout` — 60s by default — before giving
> up, instead of returning quickly. Pass a short `timeout=` or `wait=False`
> when calling these against `BonicBot.simulated()`.

Supports the context-manager form, which guarantees a `stop` on exit
(recommended for every run — matches the platform "stop in a `finally`" rule):

```python
with BonicBot("192.168.1.50") as robot:
    robot.move_forward(duration=2)
# motors stopped, socket closed, even on exception
```

Calling something this robot cannot do raises `CommandError` carrying the
robot's own explanation; a disconnect mid-call raises `RobotDisconnected`.

---

## 2. Motion (base movement)

High-level wrappers over the `drive` command. `duration=None` starts the motion
and returns immediately; a number blocks for that many seconds then stops.

**On Lite:** ✅ fully available.

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

Client-side control loops over `drive` + odometry. **Blocking**
with a timeout; the on-robot deadman backstops a stalled loop.

**On Lite:** ✅ available (`precise_motion`), but **less accurate**. Lite closes
these loops on wheel odometry alone, with no lidar or sensor fusion correcting
it — expect noticeably more drift on carpet or with wheel slip than the same
call on a Pro robot. Fine for "drive a square"; don't promise identical
precision in teaching material.

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

## 4. Navigation, mapping, locations & docking

Fire-and-monitor: goal methods start navigation; `wait_for_goal()` blocks on
`nav_status`. Coordinates are map-frame meters/radians.

> **On Lite: ❌ none of this section.** No lidar and no navigation stack, so
> every method here raises `CommandError`. This is *never*, not *not yet* — use
> `drive_distance()`/`rotate_angle()` (§3) for relative movement instead.
> `get_position()` on Lite returns dead-reckoned odometry, not a map pose (§8).

| Method | Blocks? | Description |
|---|---|---|
| `robot.go_to(x, y, theta=0.0, wait=True, timeout=60.0) -> bool` | `wait` | Navigate to a pose (Nav2). |
| `robot.navigate_waypoints(points, wait=True, timeout=60.0) -> bool` | `wait` | `points=[(x,y,theta?), ...]`. |
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
| `robot.enter_navigation_mode(name, timeout=30.0) -> bool` | yes | Tear down any nav session, launch map_server+AMCL+Nav2 localizing on saved map `name`. Raises `CommandError` if the map doesn't exist or the launch fails to come up. |
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
| `robot.delete_map(name) -> bool` | Delete a saved map and its sidecar files. Raises `CommandError` if it doesn't exist or a live navigation session is currently localized against it — `stop_nav_mode()` or switch maps first. |
| `robot.list_maps() -> list[str]` | Saved map names (the server actually returns richer metadata dicts — `list_maps()` extracts just the name; verified against the real M1 sim 2026-08-04). |
| `robot.get_map() -> dict` | Latest occupancy grid (decoded from cached `map`). |
| `robot.get_costmap() -> dict` | Latest costmap (decoded from cached `costmap`), same shape as `get_map()`. |

Named locations (semantic waypoints) — **live since 2026-08-31**. A location is
a pose in a *map's* frame, so it is stored per map: pass `map=` to work with a
map the robot isn't currently on, otherwise the running navigation session's
map is used.

| Method | Description |
|---|---|
| `robot.save_location(name, x=None, y=None, theta=0.0, map=None) -> bool` | Two forms. With `x`/`y`, saves a point picked on a map. Without them, saves **where the robot is now** — which requires it to be navigating on that map and localized, and raises `CommandError` otherwise (a pose saved before AMCL converges means nothing, and only fails much later when someone drives to it). |
| `robot.goto_location(name, wait=True, timeout=60.0, map=None) -> bool` | Navigate to a saved location — a lookup plus the normal goal path, so it reports through `nav_status` like `go_to`, and returns whether the robot got there. Raises `CommandError`, before anything moves, if the location doesn't exist or the robot isn't navigating on that map: a pose from a *different* map is a well-formed coordinate pointing at a different room. |
| `robot.list_locations(map=None) -> list[str]` | Location names, ready to hand back to `goto_location` (the server returns richer records — this extracts the name, same as `list_maps()`). Empty when there's no map to resolve. |
| `robot.get_locations(map=None) -> list[dict]` | The full records: `{"name", "x", "y", "theta"}`, sorted by name, in the map's frame. |
| `robot.delete_location(name, map=None) -> bool` / `robot.delete_all_locations(map=None) -> bool` | Manage saved locations. They're also dropped automatically when their map is deleted. |

Docking — **addon only**. A dock is a charging station the robot reverses
onto, guided by an AprilTag and a rear camera, both part of the docking
addon. On a robot without the addon, `save_dock`, `dock` and `undock` raise
`CommandError` saying docking isn't available; the simulator does the same.
Docks are saved per map, separately from locations. Dock names default to
`"default"`.

```python
robot.enter_navigation_mode("home")
# drive the robot onto the dock by hand, exactly as it should sit charging
robot.save_dock()
robot.goto_location("kitchen")
if not robot.dock():                 # False: tried, but didn't seat
    print(robot.get_dock_result())   # {"status": "failed", "error_code": ...}
```

| Method | Description |
|---|---|
| `robot.save_dock(name="default") -> bool` | Record **where the robot is parked** as this map's dock. There is no x/y form — park the robot on the dock first. Needs a navigation session and a localized robot. |
| `robot.dock(name="default", wait=True, timeout=120.0) -> bool` | Drive to the dock and reverse onto it. With `wait`, returns whether the robot ended up docked. Raises `CommandError`, before moving, with no addon, no navigation session on the dock's map, or no dock saved under `name`. |
| `robot.undock(wait=True, timeout=60.0) -> bool` | Drive straight off the dock. |
| `robot.wait_for_dock(timeout=120.0) -> bool` | Block until the current dock/undock attempt ends. |
| `robot.get_dock_status() -> str` | `idle`/`navigating`/`succeeded`/`failed`/`canceled`. |
| `robot.get_dock_result() -> dict \| None` | The latest attempt: `{"status", "goal_id"}`, plus `error` if it never started and `error_code` if it failed partway. |
| `robot.list_docks(map=None) -> list[str]` / `robot.get_docks(map=None) -> list[dict]` | Saved docks — names, or `{"name", "x", "y", "theta"}` records. Work on any robot. |
| `robot.delete_dock(name="default", map=None) -> bool` | Forget a dock. Works on any robot. |

Grouped access: `robot.nav.*`.

---

## 5. Arms, grippers & neck

Built on `servo_command` (registry camelCase joints → controller groups,
angles in **degrees** at the API boundary, converted to radians on the wire).

**Not every robot has every joint.** The 18 below are the maximum fitment
(M1). An A2 fits **7** of them — shoulder pitch and elbow per arm, one gripper
per side, and neck yaw; no wrists, no shoulder yaw/roll, **no neck pitch**. S
fits 14. It isn't even fixed per series: actuator fitment and travel limits are
per-robot config the ESP loads at boot.

You do not have to track any of that. `get_servo_angles()` reports exactly the
fitted set, and since 2026-09-04 the SDK scopes commands *and* the `wait=True`
convergence check to the joints the robot actually reports — so
`move_left_arm()` on an A2 moves the two joints it has and returns `True`,
rather than waiting for five it doesn't. Naming an absent joint explicitly is
reported by the server (`unsupported`) and skipped rather than waited on,
with a `UserWarning`. If *none* of the joints you named exist on this robot —
`open_grippers()` with no grippers fitted — it raises `CommandError`, as does
a joint name that is not a `ServoID`.

| Method | Description |
|---|---|
| `robot.set_servos(angles: dict, duration=1.0, wait=True, timeout=None) -> bool` | Set multiple joints, e.g. `{"leftElbow": 30, "neckYaw": 20}`. |
| `robot.move_left_arm(shoulder, elbow, wait=True, duration=1.0, timeout=None) -> bool` | Left arm shorthand. |
| `robot.move_right_arm(shoulder, elbow, wait=True, duration=1.0, timeout=None) -> bool` | Right arm shorthand. |
| `robot.set_grippers(left, right) -> bool` | Both grippers (degrees). |
| `robot.open_grippers() / close_grippers() -> bool` | Convenience. Command +60°/−45°, the range valid on every series (fixed 2026-09-04: these were ±90°, which no robot can reach — the command got clamped and then `wait=True` timed out waiting for a target that doesn't exist). |
| `robot.set_neck(yaw) -> bool` / `robot.look_left/right/center() -> bool` | Neck yaw. |
| `robot.reset_servos() -> bool` | Every joint **this robot has** to neutral (0°). |
| `robot.set_single_servo(joint, angle) -> bool` | One joint by name. **🔌 stub** (direct addressing) where no controller group covers it. |
| `robot.get_servo_angles() -> dict` | From `joint_states` telemetry, keyed by the same **registry camelCase** names (e.g. `"leftElbow"`) commands are sent with — not the raw snake_case URDF names the wire uses underneath. |

`ServoID` (§11) enumerates the exact 18 registry keys — the M1's real joint
set, not the old BLE-hardware set it was originally ported from.

> **Elbow angles are positive as of 2026-09-05.** Elbow travel is one-sided,
> and bonicOS-firmware `678dc38` flipped which side: it ran −50..0 (A), −90..0
> (S), −110..0 (M) and now runs 0..45 / 0..90 / 0..110, with 0 still the arm
> straight. The A number also moved from 50 to 45 in the same pass — see
> below. **Code written against the old range does not error — it clamps at 0
> and the arm never bends**, so `move_left_arm(shoulder=60, elbow=-30)` now
> means "hold the arm straight". Update saved sequences and worksheets, not
> just source. `protocol.ELBOW_RANGE_DEG` carries the range valid on every
> series.
>
> **A's 50 became 45, matching the URDF instead of the other way round.**
> `bonicbot-a2-ros` `6098b2d` had already re-signed `body.xacro` to
> `lower="0" upper="0.785"` (0..45°) ahead of the firmware catching up, rather
> than widen the URDF to the old 50°. The firmware then adopted 45 too, so
> there is one A-series elbow limit instead of two that disagreed.
>
> **The ROS lane caught up on A, not on M.** ros2_control clamps a trajectory
> to the URDF, so whichever range the URDF declares is the one that reaches
> the arm. `bonicbot-a2-ros`'s `body.xacro` (`lower="0" upper="0.785"`, axis
> negated so RViz and Gazebo still match the hardware) means **on an A2 a
> positive elbow up to 45° bends the arm, matching the range above exactly**.
> `bonicOS-m1-ros` still declares `lower="-1.9199" upper="0.0"`, so **on an M1
> the elbow does not move through the ROS path at all** — a positive angle is
> clamped to 0 by ROS, a negative one clamped to 0 by the ESP — and
> `wait=True` times out. The SDK cannot paper over that: it does not clamp
> user angles, and clamping is not what is wrong.

> **`wait=True` means the arm actually arrived, not just that the server
> acked the command** (fixed 2026-08-04 — the
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
> (fixed 2026-08-04): `move_left_arm(shoulder, elbow)`
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
> stale command "catching up" later.

Grouped access: `robot.arm.*`.

---

## 6. Head expression & display — **✅ live on A series**

Live on both Lite and Pro. On Pro the path is robot_app -> `/face/matrix_action`
-> the ros2_control plugin -> `CMD_MATRIX_ACTION` over USB CDC to the ESP.

> **Needs the base stack up.** The plugin owns `/dev/esp` and only one process
> may hold it, so with the stack down there is no path to the display at all.
> A series with no LED matrix (M1 has no subscriber for the topic) answers with
> an error rather than a silent success.

| Method | Description |
|---|---|
| `robot.set_expression(mode)` | `"normal"/"happy"/"sad"/"angry"/"surprised"/"confused"` (`HeadMode` enum). |
| `robot.look(pan=None, tilt=None, speed=None, *, duration=1.0)` | Neck pan/tilt in **degrees**, via the head controller group. |
| `robot.set_display_text(text)` | LED-matrix text (ASCII; the panel font has nothing else). |
| `robot.set_display_color(r, g, b)` | Matrix color, 0-255 per channel. |
| `robot.set_display_animation(mode)` / `play_display()` / `pause_display()` / `clear_display()` | Matrix animation control. A `DisplayAnimation` member, its bare name, or a raw firmware index. |
| `robot.set_display_brightness(value)` | Matrix brightness, **0-255** — not a 0..1 fraction. |

**Two expressions are approximations.** Firmware has no `surprised` or
`confused` face, so they show a heart and a colour effect respectively. The
robot reports the substitution and `set_expression` raises a `UserWarning`
saying which — don't build material around either without checking what the
panel actually does.

**Animation names come from `DisplayAnimation`** — `static_text`,
`scrolling_text`, `rainbow_wave`, `fire`, `plasma`, `matrix_rain`,
`custom_pattern`, `rose_color_wave`, `custom_animation`, `sad`, `love`,
`happy`, `angry`, `manual_paint`, `battery`. A bare string works, and a raw
int is passed through as a firmware animation index for anything the enum
does not name yet. An unknown name is refused, with the list the robot knows.

**A refused display command raises `CommandError`** with the robot's reason,
e.g. "no LED matrix on this series" or "the base stack is down". For an
unknown animation name, `err.result["known"]` lists the names the robot knows.

**`look` raises when nothing could move.** `tilt` is neck pitch, which an A2
does not have: `look(tilt=…)` alone raises `CommandError`, and `pan` + `tilt`
moves the pan and warns about the tilt. `speed` is accepted but ignored by the robot
(position groups take a time, not a rate) — use `duration`.

Grouped access: `robot.head.*`.

---

## 7. Speech

**On Lite:** ✅ available — BonicOS is always fitted.

| Method | Description |
|---|---|
| `robot.speak(text, voice=None, *, language=None, rate=None, engine=None, agent_id=None) -> bool` | Say `text` (at most 1000 characters). Returns once the speech is queued, not once it has been heard. |

Calls are spoken in order. If the robot can't say something it raises
`CommandError` with the reason, so a program never goes quietly silent.

### Where the words come out

The robot decides, from how it is fitted — you never pick:

| Robot | Who speaks | `engine="edge"` | `engine="cloud"` | `agent_id` |
|---|---|---|---|---|
| **With BonicOS** (the BonicOS app on the robot's tablet or phone) | the BonicOS app | ✅ the tablet's on-device voice | ✅ | ✅ |
| **Without BonicOS** (an A-series pro without the app) | the robot itself, through its own speaker | ✅ English only | ❌ | ❌ |

`engine="cloud"` and `agent_id` on a robot without BonicOS raise
`CommandError`: *"cloud voices need BonicOS, which this robot doesn't have"*.

### Options

| Option | Values | Default | Notes |
|---|---|---|---|
| `language` | a code from the tables below | the voice's own language (English, `en-US`) | Case-insensitive. |
| `rate` | `0.5` to `2.0` | `1.0` | Speaking speed; higher is faster. Works with every engine. |
| `engine` | `"edge"` or `"cloud"` | `"edge"` | `edge` is free and works offline. `cloud` is higher quality, speaks every language below, and uses the robot's credits. |
| `voice` | a cloud voice name from the table below, e.g. `"Zephyr"` | `"Zephyr"` | Cloud only: needs `engine="cloud"`. Just the name — **not** `"en-US-Chirp3-HD-Zephyr"`; the language comes from `language`. On-device voices can't be chosen. |
| `agent_id` | the id of a BonicAI agent | — | Speaks in that agent's configured voice, language and speed. `voice`, `language`, `rate` and `engine` are then ignored. BonicOS only. |

### Languages

**Without BonicOS (the robot's own voice):** English only — `language="en-US"`,
or leave `language` out. Any other code raises `CommandError`.

**With BonicOS, `engine="cloud"`:** every language below.

**With BonicOS, `engine="edge"`:** the languages below that are installed on
the tablet's on-device voice. English (`en-US`) is always installed. For any
other language, add it once on the tablet: **Settings → Accessibility →
Text-to-speech output → Speech Services by Google (⚙) → Install voice data**
(the exact path varies a little between tablets). Until then the robot raises
`CommandError` naming the missing language rather than speaking the wrong one.

| Language | Code | Language | Code |
|---|---|---|---|
| Arabic | `ar-XA` | Kannada | `kn-IN` |
| Bengali (India) | `bn-IN` | Korean | `ko-KR` |
| Bulgarian | `bg-BG` | Latvian | `lv-LV` |
| Cantonese (Hong Kong) | `yue-HK` | Lithuanian | `lt-LT` |
| Croatian | `hr-HR` | Malayalam | `ml-IN` |
| Czech | `cs-CZ` | Mandarin Chinese | `cmn-CN` |
| Danish | `da-DK` | Marathi | `mr-IN` |
| Dutch (Belgium) | `nl-BE` | Norwegian Bokmål | `nb-NO` |
| Dutch (Netherlands) | `nl-NL` | Polish | `pl-PL` |
| English (Australia) | `en-AU` | Portuguese (Brazil) | `pt-BR` |
| English (India) | `en-IN` | Punjabi | `pa-IN` |
| English (UK) | `en-GB` | Romanian | `ro-RO` |
| English (US) | `en-US` | Russian ¹ | `ru-RU` |
| Estonian | `et-EE` | Serbian | `sr-RS` |
| Finnish | `fi-FI` | Slovak | `sk-SK` |
| French (Canada) | `fr-CA` | Slovenian | `sl-SI` |
| French (France) | `fr-FR` | Spanish (Spain) | `es-ES` |
| German | `de-DE` | Spanish (US) | `es-US` |
| Greek | `el-GR` | Swedish | `sv-SE` |
| Gujarati | `gu-IN` | Tamil | `ta-IN` |
| Hebrew | `he-IL` | Telugu | `te-IN` |
| Hindi | `hi-IN` | Thai | `th-TH` |
| Hungarian | `hu-HU` | Turkish | `tr-TR` |
| Indonesian | `id-ID` | Ukrainian | `uk-UA` |
| Italian | `it-IT` | Urdu | `ur-IN` |
| Japanese | `ja-JP` | Vietnamese | `vi-VN` |

¹ Russian has only eight cloud voices: Aoede, Charon, Fenrir, Kore, Leda, Orus,
Puck and Zephyr.

### Cloud voices

Every voice speaks every language in the table above (except as noted for
Russian). Pass just the name.

| | Voices |
|---|---|
| **Female** | Achernar, Aoede, Autonoe, Callirrhoe, Despina, Erinome, Gacrux, Kore, Laomedeia, Leda, Pulcherrima, Sulafat, Vindemiatrix, Zephyr |
| **Male** | Achird, Algenib, Algieba, Alnilam, Charon, Enceladus, Fenrir, Iapetus, Orus, Puck, Rasalgethi, Sadachbia, Sadaltager, Schedar, Umbriel, Zubenelgenubi |

### Examples

```python
robot.speak("Hello!")                                    # default voice, any robot
robot.speak("Slowly now.", rate=0.8)                     # any robot
robot.speak("नमस्ते", language="hi-IN", engine="cloud")   # BonicOS: cloud, default voice
robot.speak("നമസ്കാരം", "Puck", language="ml-IN", engine="cloud")
robot.speak("வணக்கம்", language="ta-IN")                   # BonicOS: tablet voice, if Tamil is installed
robot.speak("Welcome to the lab!", agent_id="YOUR_AGENT_ID")
```

---

## 8. Sensors & telemetry

Telemetry is pushed continuously and cached; reads are **non-blocking** and
return the latest value. Use `wait_for_update()` to pace loops to the real sensor
rate.

**On Lite:** ✅ available, with two differences. `get_position()` returns
**dead-reckoned odometry** — it starts at zero and drifts, since with no map
there is no map frame; good for relative movement, not for "where am I in the
room". `get_imu()` depends on an IMU being fitted (a build option on Lite).

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

Laser scan is **off by default** — 10 Hz of ~1000 ranges is not worth carrying
for an overlay nobody has open — so it lives behind its own pair of methods
(grouped-only, `robot.sensors.*`):

| Method | Description |
|---|---|
| `robot.sensors.get_scan() -> dict \| None` | Latest scan: `{"origin": {x, y, theta}, "angle_min", "angle_increment", "range_min", "range_max", "ranges"}`, in the **map** frame. Turns the stream on for you on first call, so expect `None` for a frame or two. Also `None` if the robot has no laser, or isn't localized — scans are only emitted once the map-frame transform exists. `ranges` entries are `None` where the beam got no return. |
| `robot.sensors.get_scan_points() -> list[tuple[float, float]]` | The same scan flattened to map-frame `(x, y)` points with no-returns dropped — the convenient form for plotting, or for "is anything in front of me". |
| `robot.sensors.set_scan_enabled(enabled=True) -> bool` | Turn the stream on/off explicitly. Turning it off only stops it if no other client wants it; disconnecting counts as turning it off, so a script that forgets doesn't leak. |

**Recommended loop pattern** (from `bonic-architecture.md` §5 — never spins,
self-paces to the sensor rate):

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

Call `get_camera_frame()`/`get_frame()` and the video link comes up
transparently on first use — you never have to think about how. Frames are
**BGR `numpy` arrays** (OpenCV's native layout), same shape on every
transport and wherever your code runs. A multi-camera robot (e.g. the M1's
face and docking cameras) exposes each by name.

**On Lite:** ✅ available — but the camera is the **tablet's**, mounted on the
robot's face: one camera, at face height, pointing forward. `robot.cameras`
lists what a given robot actually has.

| Method | Description |
|---|---|
| `robot.list_cameras() -> list[str]` | Camera names from the connect handshake. Available on any transport (informational) — frames still need a video path. |
| `robot.get_camera_frame(camera=None) -> ndarray \| None` | Latest BGR frame for `camera` (default: the first), or `None` if none has arrived yet. Starts the stream on first call. |
| `robot.camera.get_frames() -> dict[str, ndarray \| None]` | Latest frame for every camera, keyed by name. |
| `robot.camera.start(cameras=None)` | Bring the stream up now instead of lazily on first `get_frame()`. Blocks until the link is established (or raises `CameraUnavailable` on timeout/no video path). |
| `robot.camera.stop()` | Tear down the video path (idempotent). Commands/telemetry are unaffected. |
| `robot.camera.pause(camera=None) -> bool` | Stop the robot **encoding** video you aren't looking at, without dropping the stream. The robot can't tell you've stopped reading frames, and an unwatched stream measured ~32% of a core on a real A2. Far cheaper than `stop()`/`start()`: the track stays attached, so there's no renegotiation and `resume()` is instant. Raises `CommandError` on a connection with no video at all. |
| `robot.camera.resume(camera=None) -> bool` | Undo `pause()`. |

If there is no video path on this connection, camera calls raise `CameraUnavailable` rather than silently returning
`None` forever.

**How it works:** two paths, picked for you, and the API is identical on both.

*Off the robot* (your laptop, anywhere on the LAN) video leaves the robot as
WebRTC media tracks, so the SDK opens its own `aiortc` peer on first use and
hands you decoded frames — continuous media, which is what a network link
wants. You never see the peer. It needs the extra deps: `pip install
bonicos[camera]` (`aiortc`, `numpy`); without them the first camera call
raises `CameraUnavailable` saying exactly that.

*On the robot* (Code Studio, or anything else running through `run_code`)
frames come back over the same connection your commands use. Code there runs
sandboxed with no network of any kind, so there is no path a WebRTC peer
could take — and pulling a frame is the cheaper route anyway: the robot is
already holding the camera's JPEG, so it sends that instead of re-encoding
video. Needs no extra install; `numpy` and OpenCV are already there.

Either way `get_camera_frame()` gives you the latest frame, and polling it
faster than the camera publishes costs nothing extra — the robot answers "you
already have this one" and you get the frame you're holding.

Grouped access: `robot.camera.*`.

### 9.1 AI — `from bonicos import ai`

Computer vision that runs **on the robot**. Every function takes a camera frame
(what `get_camera_frame` returns) — it is not tied to a robot object.

```python
from bonicos import BonicBot, ai

with BonicBot() as robot:
    cups = ai.load("cup-detector")        # a model trained in the Train tab
    while True:
        frame = robot.get_camera_frame()
        if frame is None:
            continue

        label, confidence = cups.predict(frame)[0]   # most likely class first
        if label == "cup" and confidence > 0.8:
            robot.speak("I can see a cup")

        for thing in ai.detect_objects(frame):       # built in, no training
            print(thing.label, thing.confidence, thing.center)
```

| Call | Returns | Notes |
|---|---|---|
| `ai.load(name)` | `Model` | A model trained in the Train tab, sent with this program. Case and spaces in the name are ignored. |
| `model.predict(frame)` | `[Prediction(label, confidence), …]` | Every class, most likely first; confidences sum to 1. |
| `model.classes` | `[str]` | Class names in training order. |
| `ai.list_models()` | `[str]` | Names of the trained models sent with this program. |
| `ai.detect_objects(frame, min_confidence=0.4, input_size=320)` | `[Detection]` | The 80 COCO classes (person, cup, bottle, chair, …). `input_size` 256 is faster, 416 more accurate. |
| `ai.detect_faces(frame, min_confidence=0.6)` | `[Face]` | Box + `landmarks`: `right_eye`, `left_eye`, `nose`, `mouth_right`, `mouth_left` (the person's own sides). |
| `ai.detect_markers(frame, dictionary="4x4_50")` | `[Marker]` | ArUco `id` + `corners`. Also `4x4_100`, `5x5_50`, `6x6_50`, `apriltag_36h11`. |
| `ai.detect_gestures(frame, max_hands=1)` | `[Gesture]` | `name`: `Thumb_Up`, `Thumb_Down`, `Open_Palm`, `Closed_Fist`, `Pointing_Up`, `Victory`, `ILoveYou`, or `"None"`; `hand`: the person's `"Left"`/`"Right"`; 21 `landmarks`. |

`Detection`, `Face` and `Gesture` have `x`, `y`, `width`, `height` and a
`center` in frame pixels (origin top-left). `Marker` has `corners` and `center`.

**Speed** — per frame on an A Pro (RPi 4) under the on-robot runner's CPU limit:
trained model ~25 ms, markers ~15 ms, faces ~35 ms, gestures ~100 ms, objects
~205 ms (~130 ms at `input_size=256`). A loop calling several of these runs at
the sum of their times; S Pro and M Pro are faster.

**Where it runs.** The models are part of the robot's software; trained models
reach the robot with the program that uses them. In the browser simulator, or
on a laptop, every call raises `AIUnavailable` saying so — `from bonicos import
ai` itself always succeeds. `ai.load` raises `ModelNotFound` (listing what was
sent) for an unknown name, and `InvalidModel` for a model it will not run — e.g.
one trained against a different MobileNet build.

---

## 10. System

**On Lite:** mixed. `health()` and `reconfig_wifi()` are ✅ available (the
tablet answers them). The base-session methods and the
grouped `system.get_base_session()` / `get_session_health()` are ❌ — there is
no ROS stack to supervise, so the robot answers with an `error` explaining
that. (This used to be described as gated on a `session_control` feature flag;
capability gating was removed — see PROTOCOL.md §3.1.)

| Method | Description |
|---|---|
| `robot.health() -> dict` | `{"cpu_percent", "ram_percent", "disk_percent", "temps": {sensor: °C}}`, plus `runcode` where a code runner is wired up. **Not** container state — robot_app no longer updates itself, so the host owns that; `system.update_status()` answers it. |
| `robot.restart_base_session(timeout=120.0) -> bool` | Recover a wedged robot: restart the ROS stack *underneath* mapping/navigation (drive, controllers, EKF, sensors, TF) — nav session down, base down, base up, nav session back. Refused while the robot is moving or running a nav goal — cancel/stop first. Slow (cold-start Gazebo alone is ~25s); the long default timeout reflects that, and a WebRTC video peer will drop partway through since the restart takes the camera topics with it. |
| `robot.system.start_base_session(timeout=120.0) -> bool` | Bring the base stack up. A real robot does **not** start it on boot — powering on must not energise servos on its own — so this is how a freshly-booted robot is brought to life without SSH. |
| `robot.system.stop_base_session(timeout=60.0) -> bool` | Take the base stack down; the robot can't move or perceive until it's restarted. Same guard as `restart_base_session` — refused while moving or running a goal. |
| `robot.get_session_status() -> dict` | Fresh, synchronous `{"base": {...}, "nav": {...}, "health": {...}}` — the full picture behind `system.get_base_session()`/`get_session_health()` in one round trip, without waiting for a push. |
| `robot.reconfig_wifi(ssid, password, timeout=60.0) -> bool` | Join a Wi-Fi network. Blocks until the join resolves (up to ~50 s on a wrong password); raises `CommandError` if it failed. |

Grouped-only (`robot.system.*`, not flattened onto `robot.*` — mirrors
`get_plan()`/`get_costmap()`):

| Method | Description |
|---|---|
| `robot.system.get_base_session() -> dict \| None` | Latest cached `base_session` telemetry: `{"running", "owned", "transitioning", "error"}`. `None` before the first frame arrives. |
| `robot.system.get_session_health() -> dict \| None` | Latest cached `session_health` telemetry: `{"ok", "base", "nav", "issues"}` — `issues` names the mechanism (e.g. `"amcl_not_running"`, `"pose_stale:23s"`), not just a boolean. Pushed only on change, so may still be `None` right after connecting even on a healthy robot; use `get_session_status()` for a guaranteed-fresh read. |
| `robot.system.update_status() -> dict` | Fresh read of what the robot's host knows about updates: `{"state", "phase", "percent", "version", "reported_version", "previous_version", "last_update", ...}`. `state` is `installing`, `rolling_back`, `idle`, or `unavailable` — the last meaning there is no bonic-host to ask, which is normal on a bare-metal robot, a dev laptop or the simulator and is **not** an update failure. This is the read that survives the restart an install causes. |
| `robot.system.get_update_status() -> dict \| None` | Latest cached `update_progress` telemetry, or `None` if the robot has said nothing about updates this session. Replayed on auth, so a client connecting to a robot that has just been updated — or rolled back — learns that immediately. Pushed only while an install runs. |
| `robot.system.shutdown(timeout=15.0) -> bool` | **Power the robot off.** Halts the companion computer and, where the ESP lane is reachable, cuts the power latch — so it ends up genuinely off, and someone has to press the button to bring it back. Not a stack teardown; for that see `stop_base_session()`. The ack is all there is: the process answering is the one being halted, so expect the connection to drop right after. |

Grouped access: `robot.system.*`.

---

## 11. Enums & exceptions

```python
from bonicos import HeadMode, ServoID          # enums (trimmed to core)
from bonicos import DisplayAnimation           # LED-matrix animation names
from bonicos import (
    RobotError,            # base
    ConnectionError,       # connect/handshake failed
    CommandError,          # the robot refused or failed — including "this robot can't"
    RobotDisconnected,     # link dropped mid-call
)
from bonicos.ai import (   # §9.1
    AIUnavailable,         # AI cannot run here (simulator, laptop, missing runtime)
    ModelNotFound,         # ai.load() named a model not sent with this program
    InvalidModel,          # a trained model that is damaged or does not fit this robot
)
```

`CommandError` and `RobotDisconnected` surface as **real Python exceptions**
inside user code (platform requirement) so student programs can `try/except`
them.

**Every refusal is a `CommandError`.** `err.reason` is the robot's reason,
`err.command` the refused command, and `err.result` the robot's full reply
(e.g. `known` animation names, `cameras`, per-group `failed`):

```python
from bonicos import CommandError

try:
    robot.goto_location("kitchen")
except CommandError as err:
    print(err.reason)   # "no location 'kitchen' saved on map 'home'"
```

`go_to`, `navigate_waypoints`, `goto_location`, `dock`, `undock` and the
`wait_for_*` methods return `False` when the goal was accepted but not
reached.

> **`FeatureUnavailable` no longer exists.** Capability is not advertised or
> gated (see [PROTOCOL.md](./PROTOCOL.md) §3.1) — a robot that cannot perform a
> command returns a normal `error`, which the SDK raises as `CommandError` with
> the robot's own explanation attached.

---

## 12. Worked examples

**Square patrol with obstacle awareness**

```python
from bonicos import BonicBot

with BonicBot("192.168.1.50") as robot:
    robot.wait_for_data()
    for _ in range(4):
        robot.drive_distance(1.0)
        robot.rotate_angle(90)
    robot.speak("Patrol complete")
```

**Navigate to a saved place, then gesture**

```python
with BonicBot() as robot:                       # autodiscovery
    robot.goto_location("kitchen")              # blocks until arrival — Pro only
    robot.set_expression("happy")               # A series; needs the base stack up
    robot.move_right_arm(shoulder=90, elbow=30)
```

**One program on either model** — catch the error rather than asking the robot
what it is:

```python
from bonicos import BonicBot, CommandError

with BonicBot() as robot:
    try:
        robot.go_to(2.0, 3.0)          # Pro: navigates
    except CommandError:
        robot.drive_distance(2.0)      # Lite: dead reckoning
```

**Running on the robot itself — identical code**

The on-robot runner exports `$BONICOS_HOST`/`$BONICOS_ROBOT_ID` for you, so
the same file you ran from your laptop needs no edits, and
`wait_for_update()` paces the loop to the real sensor rate:

```python
robot = BonicBot()
while robot.wait_for_update():
    robot.drive(linear_x=0.2)
    if robot.get_distance_traveled() > 2.0:
        robot.stop(); break
```

---
