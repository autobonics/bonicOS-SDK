# bonicos ⇄ robot_app — Wire Protocol

The single JSON contract between the `bonicos` SDK and its WebSocket server.
**Shared verbatim with `bonicOS-robot-app`** — both sides implement exactly
this. `bonicos/protocol.py` is the machine-readable mirror of this file.

The same protocol and router already serve WebRTC data channels in robot_app;
this document is the authority for the **local WS lane** the SDK uses, but the
message shapes are transport-identical.

> **Legend for command status (v1):**
> - **✅ live** — already implemented in `robot_app/core/command_handlers.py`.
> - **➕ new** — must be added for v1 (real implementation, ROS path exists).
> - **🔌 stub** — must be added for v1 as a **log + no-op** handler (see §8);
>   the ROS topic does not exist yet and the `bonicOS-m1-ros` team will wire it
>   later. The SDK method is fully present so user code and docs are stable.

---

## 1. Transport & connection

- **Endpoint:** `ws://<host>:8080/ws`, optionally `ws://<host>:8080/ws?robotId=<ROBOT_ID>`
- `<host>` is whoever runs the server for this model (README topology): the
  processor (`robot_app`) on **pro**, the Flutter app on **lite**. The SDK does
  not distinguish them.
- Reaching the WS handshake is the only mandatory gate — no authentication.
  The `robotId` query param is **optional**: omit it and any client on the
  LAN is accepted; supply it and it **must** equal the server's `ROBOT_ID`,
  else the server closes with code **4404**. This is a wrong-robot guard on
  a multi-robot LAN, **not** authentication.
- Messages are UTF-8 JSON text frames, one JSON object per frame.

---

## 2. Message envelope

Every message is a flat JSON object with a `type`:

```jsonc
{ "type": "<name>", "id": 42, "...": "payload fields" }
```

- **`type`** (string, required) — command name (client→robot) or event/response
  name (robot→client).
- **`id`** (int, optional) — client-assigned correlation id. If present on a
  command, the robot echoes it on the matching `ack`/`error`. High-rate commands
  (`drive`) omit it and get no reply.
- Payload fields are flat alongside `type` (no nested `payload` object).

### Responses

| type | when | shape |
|---|---|---|
| `ack` | command accepted / completed synchronously | `{ "type":"ack", "id":42, ...result }` |
| `error` | command rejected or failed | `{ "type":"error", "id":42, "error":"<reason>" }` |

**Two response types, not three.** There is no `feature_unavailable`. A command
this robot cannot perform is an `error` like any other failure — see §3.1 for
why capability is not negotiated, and §2.1 for what the error must say.

`error` reasons currently in use: `not_authenticated`, `rate_limited`,
`unknown_command:<type>`, `invalid_json`, plus handler-specific strings.

### 2.1 Error text is the contract

Because clients do not predict what a robot can do (§3.1), **the server's error
string is the entire user experience** for an unsupported operation. It is the
only thing standing between a user and a robot that silently did nothing. Three
rules, enforced in each server's router rather than per handler:

1. **Never `unknown_command` for a command that exists in this document.**
   `unknown_command` means *"I have never heard of this message type"* — it reads
   as a version mismatch and sends people to debug the wrong layer. A robot that
   understands `nav_goal` but has no lidar must not use it.
2. **State the reason, not the mechanism.** `"this robot has no navigation — no
   lidar or on-board computer"` beats `"feature_unavailable: navigation"`.
3. **Point somewhere.** Name the doc (`LITE.md`, `API.md`) so the reader can find
   the full capability matrix.

> **Phase 2 — dynamic, robot-specific errors.** Today these strings are
> hardcoded per server, and each server knows only what its *whole class* of
> robot supports (`robot_app` ⇒ pro, the Flutter app ⇒ lite). The planned next
> step is for each server to read a **local per-robot config file at startup**
> — series, fitted joints, LED matrix present, IMU present, encoders — and
> generate errors from it, so a 10-servo robot can say *"this robot has no
> `leftWristYaw`"* rather than timing out. See §3.2.

---

## 3. Auth handshake

First message on a gating lane is `auth`; on the local WS lane the WS
handshake (+ optional `robotId` match) already gated the connection, so
`auth` is **accepted and ignored** (sent anyway for wire-compat and
forward-compat with v1 proximity auth).

```jsonc
// client → robot
{ "type": "auth", "token": "<BONICOS_TOKEN or empty>", "uid": "optional",
  "protocol_version": 1 }

// robot → client
{ "type": "auth_result", "ok": true,
  "robot_id": "M1_001",
  "series":   "M",
  "cameras":  ["face", "docking"] }
```

- **v0 (current): open.** No token verification; access is gated platform-side.
  The SDK still sends `token` if `BONICOS_TOKEN` is set.
- After `auth_result` the server **replays cached `map`/`costmap`** so a client
  joining mid-session renders immediately.
- `protocol_version` — see §9.

**`auth_result` is not optional and must not be delayed.** The SDK's
`WebSocketTransport.connect()` blocks on it before returning, so a server that
never sends it hangs every client at connect. It is the session-start signal.

### 3.1 The handshake is a session start, not a capability negotiation

`auth_result` tells a client **who it is connected to**. It deliberately does
**not** tell a client what that robot can do.

| Field | Meaning | Client use |
|---|---|---|
| `robot_id` | this robot's id | display, wrong-robot guard |
| `series` | `"A"` / `"S"` / `"M"` — the chassis family | display only |
| `cameras` | ordered camera names this robot streams | `get_camera_frame(name)` |

`cameras` earns its place because it is a **name set a client cannot guess** —
`get_camera_frame("face")` needs to know that string exists before a video peer
is negotiated. An empty list means no camera. It is an enumeration, not a flag.

**Clients are optimistic: send the command, handle the error.** A client holds
no model of the robot's capabilities, performs no local gating, and never
predicts a failure. If a robot cannot do something, it says so (§2.1).

#### Why there is no `features` map

Earlier revisions advertised a `features: {navigation: bool, ...}` map, and both
the SDK and each server gated commands against it. That is removed. The reasons
are worth recording, because the design goal it served is still correct:

- **It encoded a binary as a matrix.** Every key that mattered
  (`navigation`, `mapping`, `locations`, `session_control`, `run_code`,
  `moveit`, `depth_camera`) was true precisely when the robot had an on-board
  computer. It was `variant == "pro"` written a dozen times.
- **It required three synchronized mirrors** — the Python SDK, `robot_app`, and
  the Dart lite server — plus a completeness test to keep them honest. Mirrors
  drift. In practice they did: the two sides disagreed on whether an *absent*
  key meant allowed or denied.
- **A gate can be wrong about its own hardware; missing code cannot.** The lite
  server does not need a table to know it cannot navigate — it has no `nav_goal`
  handler. That is a stronger correctness property, obtained by deletion.
- **The extensibility goal is preserved.** The rule "never bake a capability
  table into a client" was right, and it is now satisfied *maximally*: clients
  contain no capability table at all, so a new model needs no client release.

**Capability is documented, not negotiated.** The per-model matrix lives in
[`LITE.md`](./LITE.md) and the Lite column of [`API.md`](./API.md). A person
writing a program knows which robot they own; that is a reasonable thing to
expect, and it is far cheaper than three mirrored tables and a test suite.

#### Consequences to be aware of

- **Failure moves from call time to round-trip time** (~35–80 ms later). Against
  a 5 s default ack timeout this is immaterial.
- **Cached readers are ambiguous.** `get_map()` on a robot with no mapping
  returns `None`, which is indistinguishable from *"nothing has arrived yet"*.
  Documented rather than mechanised.
- **Stub handlers (§8) become more dangerous**, because there is no
  machine-readable signal to cross-check them against. See §8.

### 3.2 Phase 2 — per-robot config, and what it will *not* change

Each server currently knows only what its whole class of robot supports.
The planned next step gives each server a **local configuration file, read once
at startup**, describing that individual robot: series, fitted joints, LED
matrix, IMU, encoders, cameras. Errors are then generated from it.

Two constraints on that work, decided in advance:

1. **The config is local. It is never read from Firebase or any network
   source.** A robot must know what it is with no connectivity — an offline
   robot that cannot report its own joint count is broken. Provisioning writes
   the file; the server reads it at startup.
2. **It does not come back into the handshake.** The server answers questions at
   command time; it does not advertise upfront. Adding fields back to
   `auth_result` would restore the client-side model this revision removed.

`robot_app` already has the seed of this (`robot_config.yaml` → id, series,
code), as does the lite server (fitted joints discovered from the BLE
`RESP_BATTERY` online-servo array — local, from hardware, exactly the right
pattern). Neither is wired to error generation yet.

**Known gap until then.** Naming a joint the robot does not physically have is
the one case that fails *silently*: `set_servos(wait=True)` waits for
`joint_states` convergence on an actuator that will never move, and times out
with no explanation. Two mitigations that need no server change: the timeout
message should name the joint and ask whether it is fitted, and
`get_servo_angles()` already returns exactly the fitted set (it is derived from
telemetry), so it is the runtime way to discover a robot's joints today.

---

## 4. Completion model (how the SDK blocks)

Three patterns; the SDK's blocking methods are built on them.

1. **Immediate** — handler returns a result; server sends `ack` with the result
   fields. `save_map`, `list_maps`, `health`, servo commands. The SDK's
   `wait_for_ack(id)` returns it.
2. **Fire-and-monitor** — long actions (Nav2). `ack` returns a `goal_id`
   *immediately*; real completion arrives as `nav_status` telemetry events
   (`navigating → succeeded | failed | canceled`). The SDK's `wait_for_goal()`
   watches `nav_status`.
3. **Client-side loop** — precise motion (`drive_distance`, …). No dedicated
   server command in v1; the SDK loops `drive` + odom locally.

There is **no** generic per-command "completion event" in v1; use the pattern
above per command. (A future `wait_for_completion(cmd_id)` may unify this.)

---

## 5. Commands (client → robot)

Grouped by area. Fields shown are the payload alongside `type`.

### 5.1 Motion — `stream`/high-rate

| type | status | fields | reply |
|---|---|---|---|
| `drive` | ✅ live | `linear_x` (m/s), `angular_z` (rad/s) | none (high-rate) |

`stop`, `move_forward`, `turn_left`, etc. are **SDK-side conveniences** that emit
`drive` frames — they are *not* separate wire commands. `stop` = `drive` with
zeros. The **cmd_vel deadman** (400 ms) makes continuous `drive` safe.

### 5.2 Navigation & mapping

| type | status | fields | reply / result |
|---|---|---|---|
| `nav_goal` | ✅ live | `x`, `y`, `theta?` | `ack {goal_id}`, then `nav_status` events |
| `navigate_through_waypoints` | ✅ live | `waypoints:[{x,y,theta?}]` | `ack {goal_id}`, then `nav_status` |
| `cancel_nav` | ✅ live | — | `ack {canceled: bool}` |
| `set_initial_pose` | ➕ new | `x`, `y`, `theta?` | `ack {ok}` |
| `start_navigation` | 🔌 stub | — | `ack {ok}` (Nav2 lifecycle bring-up) |
| `stop_navigation` | 🔌 stub | — | `ack {ok}` |
| `enter_mapping_mode` | ✅ live | — | `ack {ok, mode, error?}` |
| `enter_navigation_mode` | ✅ live | `name` | `ack {ok, mode, map, error?}` |
| `stop_nav_mode` | ✅ live | — | `ack {ok, mode, error?}` |
| `get_nav_mode` | ✅ live | — | `ack {mode, map, transitioning, localized}` |
| `start_mapping` | ✅ live | — | `ack {ok}` |
| `stop_mapping` | ✅ live | — | `ack {ok}` |
| `save_map` | ✅ live | `name?` | `ack {ok, name}` |
| `load_map` | ✅ live | `name` | `ack {ok, name}` |
| `delete_map` | ✅ live | `name` | `ack {ok, name, error?}` |
| `list_maps` | ✅ live | — | `ack {maps:[{name,size,modified},...]}` — metadata dicts, not plain names (verified against real M1 sim 2026-08-04); the SDK's `list_maps()` extracts just `name` to honor its `List[str]` contract |

**Nav-mode session switching** (`enter_mapping_mode`/`enter_navigation_mode`/
`stop_nav_mode`/`get_nav_mode`, added 2026-08-08) is distinct from
`start_mapping`/`stop_mapping`: those two just pause/unpause slam_toolbox's
`paused_new_measurements` param *inside* an already-running mapping session
(idempotent — `start_mapping` always ends paused=false, `stop_mapping` always
ends paused=true, regardless of prior state). `enter_mapping_mode`/
`enter_navigation_mode` instead launch or kill the **whole ROS nav launch
tree** (`bonicbot_m1_nav`'s `mapping.launch.py` / `navigation.launch.py`, one
`ros2 launch` process group per session, owned by robot_app's
`NavModeManager`) — slow (multi-second settle time; the SDK's default
`timeout=30.0`/`15.0` on these calls, vs 5s elsewhere) because a process tree
has to come up or be torn down. Typical mapping workflow:
`enter_mapping_mode()` → `start_mapping()` → drive around → `stop_mapping()`
→ `save_map(name)` → `enter_navigation_mode(name)`.

`enter_navigation_mode` returns `ok:false` if the named map doesn't exist on
disk or the launch exits during startup — check `error` for why.
All of `enter_mapping_mode` / `enter_navigation_mode` / `start_mapping` /
`nav_goal` require a navigation stack, so a robot without one (any lite model)
answers them with an `error` naming the reason (§2.1). `stop_nav_mode` and
`get_nav_mode` are safe everywhere — `get_nav_mode` reports `"idle"` rather
than failing. On boot, robot_app
auto-resumes the last **navigation** session (not mapping — a fresh mapping
session can't restore a partial map) if its map still exists on disk.

`delete_map` refuses (`ok:false`) a map that doesn't exist, or the map a live
navigation session is currently localized against.

**AMCL initial-pose seeding (added 2026-08-09):** unlike slam_toolbox, AMCL
publishes nothing — no `map -> odom`, so the `map` frame doesn't exist and
every Nav2 goal fails to plan — until it's told where the robot is.
`enter_navigation_mode` and `load_map` (an in-place map swap) therefore both
auto-seed AMCL server-side: the last pose remembered on that map (persisted
across sessions), or the map's origin if never visited. `/initialpose` is a
plain (volatile) topic, so a pose published before AMCL has subscribed is
simply dropped — the server retries on a `NAV_SEED_INTERVAL_S` cadence
against a `NAV_SEED_TIMEOUT_S` wall-clock deadline (default 60s; a loaded
host can take much longer than a fixed attempt count would tolerate), not a
fixed retry count. **This can fail** — the deadline can be missed on a slow
host, or nothing has ever been recorded for a brand-new map's neighborhood —
which is exactly what `nav_mode`'s `localized` field (§6) reports;
`set_initial_pose` remains available to place the robot by hand when it does.

### 5.3 Named locations (semantic waypoints)

Back the education "go to the kitchen" workflow. All 🔌 stub in v1 — they map to
`/robot/*` topics/services in `bonicOS-m1-ros` that are not implemented yet.

| type | status | fields | reply |
|---|---|---|---|
| `save_location` | 🔌 stub | `name` (save current pose) | `ack {ok, name}` |
| `goto_location` | 🔌 stub | `name` | `ack {goal_id}`, then `nav_status` |
| `delete_location` | 🔌 stub | `name` | `ack {ok}` |
| `delete_all_locations` | 🔌 stub | — | `ack {ok}` |
| `list_locations` | 🔌 stub | — | `ack {locations:[...]}` (returns `[]` while stubbed) |

### 5.4 Servos / arms / grippers / neck

| type | status | fields | reply |
|---|---|---|---|
| `servo_command` | ✅ live | `servos:{<camelCaseJoint>: rad, ...}`, `duration?` | `ack {ok, groups, unknown}` |
| `servo_single` | 🔌 stub | `joint`, `angle`, `speed?`, `acc?` | `ack {ok}` |

`servo_command` already maps registry camelCase joint keys → snake_case URDF
joints, groups them per ros2_control controller, and publishes one command per
group. Arm/gripper/neck convenience methods in the SDK build `servo_command`
payloads. `servo_single` (direct addressing, carried from the BLE SDK) is a stub
until a matching topic exists.

**The full registry (all 18 M1 joints)** — mirrored verbatim as
`bonicos/protocol.py`'s `JOINT_NAME_MAP` and `bonicOS-robot-app/app/config.py`'s
`ACTUATOR_JOINTS`, verified round-trip end-to-end against the real M1 sim
(2026-08-04). `joint_states` telemetry (§6) reports the snake_case name, never
the camelCase key:

| camelCase key (`servos` dict) | snake_case URDF joint |
|---|---|
| `rightGripper` | `right_gripper_finger1_joint` |
| `rightGripperYaw` | `right_gripper_yaw_joint` |
| `rightWristPitch` | `right_wrist_pitch_joint` |
| `rightWristYaw` | `right_wrist_yaw_joint` |
| `rightElbow` | `right_elbow_joint` |
| `rightShoulderYaw` | `right_shoulder_yaw_joint` |
| `rightShoulderRoll` | `right_shoulder_roll_joint` |
| `rightShoulderPitch` | `right_shoulder_pitch_joint` |
| `leftShoulderPitch` | `left_shoulder_pitch_joint` |
| `leftShoulderRoll` | `left_shoulder_roll_joint` |
| `leftShoulderYaw` | `left_shoulder_yaw_joint` |
| `leftElbow` | `left_elbow_joint` |
| `leftWristYaw` | `left_wrist_yaw_joint` |
| `leftWristPitch` | `left_wrist_pitch_joint` |
| `leftGripperYaw` | `left_gripper_yaw_joint` |
| `leftGripper` | `left_gripper_finger1_joint` |
| `neckYaw` | `neck_yaw_joint` |
| `neckPitch` | `neck_pitch_joint` |

### 5.5 Head expression & LED matrix — all 🔌 stub

No ROS path exists yet (these were BLE-only on the old `bonicbot`). Add as
log + no-op handlers so the API is complete.

| type | status | fields |
|---|---|---|
| `head_mode` | 🔌 stub | `mode` (`normal`/`happy`/`sad`/`angry`/`surprised`/`confused`) |
| `head_look` | 🔌 stub | `pan?`, `tilt?`, `speed?` |
| `display_text` | 🔌 stub | `text` |
| `display_color` | 🔌 stub | `r`, `g`, `b` |
| `display_animation` | 🔌 stub | `mode` |
| `display_brightness` | 🔌 stub | `value` |
| `display_clear` | 🔌 stub | — |

> `head_look` (pan/tilt) *may* be realizable through `servo_command`'s head
> controller group where a robot has head servos — implementers should prefer
> that path over a stub where the controller exists; keep the wire command
> name stable regardless.

### 5.6 Speech — `speak` (model-topology-aware)

**One wire command, three execution paths chosen server-side** from series
config + whether a tablet is attached. The SDK just sends `speak`.

| type | status | fields | reply |
|---|---|---|---|
| `speak` | ➕ new | `text`, `voice?` | `ack {ok}` |

Server-side routing (see also README topology table):

| Model / config | Where `speak` executes | robot_app action |
|---|---|---|
| **Lite** (always has tablet) | Android TTS | *robot_app is not the server here* — the Flutter WS server handles `speak` directly |
| **Pro + tablet** | Android TTS | relay `text` to the tablet **via ESP32** (a `/esp/*` topic) — 🔌 stub until firmware/ROS wire it |
| **Pro, no tablet** (a2-pro) | Pi's own TTS | invoke on-device TTS — 🔌 stub until the TTS node/service exists |

Amplifier ownership (processor vs tablet) is decided when the user enters
**developer mode** and is out of scope for the wire protocol — `speak` behaves
the same regardless; only *who drives the amplifier* changes underneath.

### 5.7 System & session

| type | status | fields | reply |
|---|---|---|---|
| `health` | ✅ live | — | `ack {type:"health", cpu, ram, temp, ...}` |
| `restart_base_session` | ✅ live | — | `ack {ok, error?, running, transitioning}` |
| `get_session_status` | ✅ live | — | `ack {base:{...}, nav:{...}, health:{...}}` |
| `reconfig_wifi` | ✅ live | `ssid`, `password` | `ack {ok}` |
| `trigger_update` | ✅ live | — | `ack {ok, detail}` |
| `subscribe` | ✅ live | `events:[...]` (omit/empty ⇒ all) | `ack {ok, events:[...]}` |
| `llm_query` | ✅ live | `prompt`, `model?` | stream of `llm_token` events (display only) |

`subscribe` narrows the telemetry firehose per client and replays cached
`map`/`costmap` for newly-covered events.

**Base session supervision** (added 2026-08-09, after a ~15min outage that
stayed invisible to any client — see `SESSION_SUPERVISION.md` in
`bonicOS-robot-app`): robot_app now supervises the ROS stack *underneath*
mapping/navigation too — drive, controllers, EKF, sensors, TF — not just the
nav session on top of it. `restart_base_session` is the operator-facing
recovery action for a wedged robot (nav down → base down → base up → nav
back); start/stop aren't exposed separately because a bare stop leaves a
robot recoverable only over SSH. It requires a supervised ROS stack, so it
exists on pro only; it is refused (`ok:false`) while the robot is under manual
drive or running a nav goal — cancel/stop that first, the server will not do it
for you. It's also
**slow**: a cold Gazebo start alone is ~25s, on top of nav teardown and an
AMCL reseed — the SDK's default timeout on this call is 120s, far above the
5s baseline, and a WebRTC video peer will drop partway through since the
restart takes the camera topics with it. `get_session_status` is a
synchronous, ungated point-in-time read of `{base, nav, health}` — the same
state `base_session`/`session_health` telemetry (§6) push on change, in one
round trip without waiting for a push.

> **Status (2026-08-09, per `SESSION_SUPERVISION.md`): built, not yet
> hardware-verified.** The base-session feature depends on robot_app running
> as a persistent service (`deploy/bonic-robot-app.service`) rather than a
> child of the session scripts — without that, `restart_base_session` tearing
> its own process's supervisor down along with the base stack is a real risk.
> Confirm the deployment model before relying on this command against a given
> robot.

---

## 6. Telemetry & async events (robot → client)

Pushed continuously to subscribed clients; the SDK caches the latest of each and
exposes them through `read_telemetry()` / sensor getters. Event names and payload
shapes come from `robot_app/ros/bridge_base.py`.

| event | payload | SDK surface |
|---|---|---|
| `pose` | `x, y, theta` | `get_position()`, `get_x/y/heading()` |
| `odom` | `x, y, theta, vx, vtheta` | precise-motion loop, `is_moving()` |
| `battery` | `voltage, current, soc` | `get_battery()` |
| `joint_states` | `name:[...], position:[...]` | `get_servo_angles()` |
| `imu` | `ax, ay, az, gx, gy, gz` | `get_imu()` |
| `map` | `info:{...}, data_b64` (zlib) | `get_map()` (cached, replayed on auth) |
| `costmap` | `info:{...}, data_b64` | cached, replayed on auth |
| `plan` | `points:[[x,y],...]` | `get_plan()` |
| `nav_status` | `status: idle\|navigating\|succeeded\|failed\|canceled`, `goal_id?`, `distance_to_goal?` | `wait_for_goal()`, `get_nav_status()` |
| `nav_mode` | `mode: idle\|mapping\|navigating`, `map`, `transitioning`, `localized` | `get_nav_mode()` (cached, replayed on auth — same mechanism as `map`/`costmap`) |
| `base_session` | `running, owned, transitioning, error` | `system.get_base_session()` (cached, replayed on auth) |
| `session_health` | `ok, base:{...}, nav:{...}, issues:[...]` | `system.get_session_health()` (cached, replayed on auth; pushed only on change) |
| `llm_token` | `token, done` | `llm_query()` streaming |

**`pose` is TF-derived, not topic-derived (changed 2026-08-07).**
`robot_app` looks up `map_frame -> base_frame` TF (`RosBridge._poll_pose`,
polled at `POSE_RATE_HZ`, default 15Hz) instead of subscribing to a
localization topic — a single topic (`/amcl_pose` for AMCL, slam_toolbox's
`/pose`) only exists in one of mapping/navigation mode, whereas the
localizer's `map -> odom` broadcast (and hence the full TF chain) is present
in both. Before this change `pose` silently stayed at its default
`(0, 0, 0)` in any setup without AMCL actively localizing (e.g. this SDK's
Gazebo sim, mapping mode); `get_position()`/`get_x/y/heading()` should now
return real values whenever a localizer (AMCL or slam_toolbox) is up,
regardless of mode. `drive_distance`/`rotate_angle` (`precise_motion.py`)
were unaffected either way — they use `odom`, not `pose`.

**`nav_mode.localized` is freshness-checked, not latched (added 2026-08-09).**
Earlier, `localized` was set once by the AMCL-seeding loop and never
revisited, so AMCL crashing or TF flapping left it silently reporting `True`
forever — this is what let the 2026-08-09 outage stay invisible to clients.
robot_app now re-derives it from `pose` staleness (`POSE_STALE_AFTER_S`,
default 15s) on a poll timer, so a client watching `nav_mode` learns when
localization is actually lost, not just when it was first seeded.

**`base_session`/`session_health` are new telemetry classes (2026-08-09)**,
distinct from `nav_mode`: they report the ROS stack *underneath*
mapping/navigation (drive, controllers, EKF, sensors, TF) rather than the
mapping/navigation session itself. `session_health.issues` names the
mechanism (e.g. `"amcl_not_running"`, `"pose_stale:23s"`,
`"clock_publishers=2"`, `"base_stack_down"`), not just a pass/fail boolean —
see `SESSION_SUPERVISION.md` in `bonicOS-robot-app` for the outage that
motivated naming the mechanism. Both are cached and replayed like `map`, but
`session_health` is pushed only on change (a healthy robot is silent on the
wire) — use `get_session_status()` (§5.7) for a guaranteed-fresh read
instead of waiting on a push.

**Deferred events** (later phase, not v1): `vision/*` detections, camera frames.

---

## 7. Rate limiting

The server applies a per-client token bucket (currently 30 messages / 5 s) to
non-`drive` commands. Exceeding it yields `error: rate_limited`. The SDK should
not batch-spam acked commands; `drive` is exempt as high-rate.

---

## 8. Stub handler convention (server side)

For every 🔌 stub command, `robot_app` ships a handler now that:

1. logs at debug: `log.debug("STUB %s — no ROS path yet: %r", type, msg)`;
2. returns a **successful-shaped** `ack` (e.g. `{ "ok": true }`, empty lists for
   list-style commands) so SDK code and student programs run end-to-end;
3. carries a `# TODO(m1-ros): call <expected rclpy topic/service>` marker naming
   the exact future ROS resource (see `bonicbot-bridge` for the `/robot/*`,
   `/vision/*`, controller topic names to target).

This keeps the **SDK API and this protocol frozen** while the robot side catches
up: swapping a stub for a real implementation is a server-only change, invisible
to `bonicos` and to user code.

### 8.1 Stub vs. never — the distinction is now documentation-only

There are two reasons a command might not do anything, and they call for
opposite responses:

| | **Stub** (🔌) | **Never** |
|---|---|---|
| Means | *not yet* — the hardware exists, the wiring doesn't | *structurally absent* — there is no lidar to add |
| Response | success-shaped `ack`, no-op | `error` with a reason (§2.1) |
| The same program later | works untouched once the handler lands | will never work on this robot |
| Example | `display_text` on pro | `nav_goal` on lite |

Getting this backwards is costly in both directions. Acking `nav_goal` on a lite
robot leaves someone watching a stationary robot while debugging correct code.
Erroring on a pro stub breaks a program that would have started working on its
own after a server update.

> **This got riskier when `features` was removed (§3.1), and that is an accepted
> trade.** A stub previously had a machine-readable counterpart a client could
> cross-check; now the *only* record that `display_text` is a no-op on pro is
> this document and [`API.md`](./API.md). Someone watching a blank LED matrix
> has no runtime way to tell "not implemented yet" from "my code is wrong".
> **Keeping the 🔌 markers accurate is therefore load-bearing, not cosmetic** —
> when a stub becomes real, updating `API.md` is part of the change.

---

## 9. Versioning

- `PROTOCOL_VERSION` (int) lives in `bonicos/protocol.py` and is sent as
  `protocol_version` in the `auth` message.
- The server compares and **warns on mismatch** (and may reject on a major
  break). Once `bonicos` is on PyPI the server no longer controls which client
  version is in the field, so the handshake must carry the version.
- Bump the integer on any breaking change to a command/event shape in this file;
  additive commands/fields do **not** bump it.

---

## 10. Command surface parity notes (for implementers)

The two SDKs being replaced touch a wider surface than v1 ships. For traceability
when later phases land:

- **Deferred `/vision/*`** (YOLO/face/pose/gesture/ArUco enable/disable + result
  topics) — in `bonicbot-bridge/vision.py`.
- **Deferred `/robot/*` exploration** (`start_explore`, `explore/status`, …) — in
  `bonicbot-bridge/autonomous.py`.
- **Deferred sequences / camera capture** (speak's siblings from the app bridge)
  — in `Bonicbot-SDKs/bonicbot` app-bridge features.

None are part of the v1 wire protocol; listed so nobody assumes they were
forgotten.
