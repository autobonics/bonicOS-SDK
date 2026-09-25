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
| `ack` with `ok: false` | the handler refused or failed | `{ "type":"ack", "id":42, "ok":false, "error":"<reason>", ...result }` |
| `error` | command rejected or failed | `{ "type":"error", "id":42, "error":"<reason>" }` |

**Two response types, not three.** There is no `feature_unavailable`. A command
this robot cannot perform is refused like any other failure — see §3.1 for
why capability is not negotiated, and §2.1 for what the refusal must say.

**A refusal is either an `error` or an `ack` with `ok: false`.** Both carry
the reason in `error` (some replies use `detail`), and an `ok: false` ack may
carry more fields (`known` names, `cameras`, per-group `failed`). `bonicos`
raises both as `CommandError(command, reason, result)`. An `ack` with no `ok`
field is success — `nav_goal`'s `{goal_id}`, `list_maps`'s `{maps}`. One
exception: `update_status` with `state: "unavailable"` is returned, not
raised.

`error` reasons currently in use: `not_authenticated`, `rate_limited`,
`unknown_command:<type>`, `invalid_json`, plus handler-specific strings.

### 2.1 Error text is the contract

Because clients do not predict what a robot can do (§3.1), **the server's error
string is the entire user experience** for an unsupported operation, in
either refusal shape (§2). It is the
only thing standing between a user and a robot that silently did nothing. Three
rules, enforced in each server's router rather than per handler:

1. **Never `unknown_command` for a command that exists in this document.**
   `unknown_command` means *"I have never heard of this message type"* — it reads
   as a version mismatch and sends people to debug the wrong layer. A robot that
   understands `nav_goal` but has no lidar must not use it.
2. **State the reason, not the mechanism.** `"this robot has no navigation — no
   lidar or on-board computer"` beats `"feature_unavailable: navigation"`.
3. **Point somewhere.** Name the doc (`API.md`) so the reader can find the full
   capability matrix.

Today these strings are hardcoded per server, and each server knows only what
its *whole class* of robot supports (`robot_app` ⇒ pro, the Flutter app ⇒ lite),
so they cannot yet name a specific missing part on an individual robot.

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
  "series":   "m",
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
| `series` | `"a"` / `"s"` / `"m"` — the chassis family | display only |
| `cameras` | ordered camera names this robot streams | `get_camera_frame(name)` |

`cameras` earns its place because it is a **name set a client cannot guess** —
`get_camera_frame("face")` needs to know that string exists before a video peer
is negotiated. An empty list means no camera. It is an enumeration, not a flag.

**Clients are optimistic: send the command, handle the error.** A client holds
no model of the robot's capabilities, performs no local gating, and never
predicts a failure. If a robot cannot do something, it says so (§2.1).

**Capability is documented, not negotiated.** The per-model matrix lives in
[`API.md`](./API.md), where every section carries an **On Lite** line. A person
writing a program knows which robot they own. There is no `features` map in the
handshake, no client-side gate, and none should be added — a server with no
handler for a command cannot be wrong about its own hardware the way a
capability table can.

#### Consequences for client authors

- **Failure arrives at round-trip time, not call time** (~35–80 ms later).
  Against a 5 s default ack timeout this is immaterial.
- **Cached readers are ambiguous.** `get_map()` on a robot with no mapping
  returns `None`, which is indistinguishable from *"nothing has arrived yet"*.
- **Naming a joint the robot does not physically have fails silently** — it is
  the one case with no error. `set_servos(wait=True)` waits for `joint_states`
  convergence on an actuator that will never move, and times out with no
  explanation. `get_servo_angles()` returns exactly the fitted set, so it is the
  runtime way to discover a robot's joints.

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

> **One command this document covers and the SDK deliberately does not:**
> `run_code` / `run_code_cancel` (with their `run_code_output` and
> `run_code_result` events). They hand Python source to the robot to execute
> against `bonicos` — so the code they carry is an SDK *caller*, and a
> `robot.run_code()` would be the SDK asking a robot to run the SDK. They
> exist because a school Chromebook cannot install Python, not because the
> SDK needs them; `bonicOS-robot-app`'s `RUNCODE_IMPLEMENTATION.md` is their
> spec. Everything else in §5 has a method.

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

Back the education "go to the kitchen" workflow. **Live since 2026-08-31**
(previously stubs), backed by `robot_app`'s `managers/location_store.py`.

A location is a pose in a **map's** coordinate frame, so every command here
resolves a map first: the `map` field if given, otherwise whichever map the
running navigation session has loaded. No map and no session ⇒ refused.
Passing `map` explicitly is what lets a UI list or tidy up places saved on a
map the robot is not currently using.

| type | status | fields | reply |
|---|---|---|---|
| `save_location` | ✅ live | `name`, `map?`, `x?`, `y?`, `theta?` | `ack {ok, name, map, x, y, theta}` |
| `goto_location` | ✅ live | `name`, `map?` | `ack {ok, name, map, goal_id}`, then `nav_status` |
| `delete_location` | ✅ live | `name`, `map?` | `ack {ok, name, map}` |
| `delete_all_locations` | ✅ live | `map?` | `ack {ok, map, deleted}` |
| `list_locations` | ✅ live | `map?` | `ack {ok, map, locations:[{name,x,y,theta},...]}` |

**`list_locations` returns records, not names.** Same shape trap as
`list_maps` — `nav.list_locations()` extracts the names, `nav.get_locations()`
keeps the coordinates.

`save_location` has two forms. With `x`/`y` it stores a point the operator
picked on a map, which need not be the loaded one. Without them it stores where
the robot is standing — and *that* form is refused unless the robot is
navigating on the map being written to, because saving "here" while AMCL has
not converged records a coordinate that means nothing and only fails much
later, when someone navigates to it.

`goto_location` is pinned to the loaded map for a sharper reason: a map-frame
pose from a *different* map is a perfectly well-formed coordinate pointing at a
different room, and nothing downstream would catch it. The robot would simply
drive there.

Locations are dropped when their map is deleted (`delete_map`) — a later map
reusing that name must not inherit places from a different room.

### 5.3.1 Docking — addon only

A dock is a charging station the robot reverses onto, guided by an AprilTag
and a rear camera, both part of an optional per-robot **docking addon**.

| type | status | fields | reply |
|---|---|---|---|
| `save_dock` | ✅ live (addon) | `name?` (default `"default"`), `map?` | `ack {ok, name, map, x, y, theta}` |
| `list_docks` | ✅ live | `map?` | `ack {ok, map, docks:[{name,x,y,theta},...]}` |
| `delete_dock` | ✅ live | `name?`, `map?` | `ack {ok, name, map}` |
| `dock` | ✅ live (addon) | `name?`, `map?` | `ack {ok, name, map, goal_id}`, then `dock_status` |
| `undock` | ✅ live (addon) | — | `ack {ok, goal_id}`, then `dock_status` |

A robot without the addon answers `save_dock`, `dock` and `undock` with
`ok: false` and an `error` saying docking isn't available (§3.1: the client
sends, the robot refuses). `list_docks`/`delete_dock` answer on every robot.
`health` includes `capabilities: {"docking": bool}` for display only; clients
do not gate commands on it.

A dock pose is map-frame and resolves its map exactly as locations do, but is
stored separately — docks never appear in `list_locations`. `save_dock` has
**no x/y form**: it records where the robot is parked. `dock` and `save_dock`
require a navigation session on that map.

`dock`/`undock` ack once the goal is accepted. The ack can take several
seconds while the robot starts its docking pipeline; `bonicos` waits up to
30 s.

### 5.4 Servos / arms / grippers / neck

| type | status | fields | reply |
|---|---|---|---|
| `servo_command` | ✅ live | `servos:{<camelCaseJoint>: rad, ...}`, `duration?` | `ack {ok, groups, failed, unknown, unsupported}` |
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

#### The registry is a vocabulary, not an inventory

Those 18 are the **maximum** fitment. Which of them a robot in front of you
actually has is a different question, and the answer is not even fixed per
series:

| series | fitted | what's missing |
|---|---|---|
| A (A2) | 7 — shoulder pitch + elbow per arm, one gripper per side, neck yaw | wrists, shoulder yaw/roll, gripper yaw, **neck pitch** |
| S | 14 | gripper yaw, wrist pitch |
| M (M1) | all 18 | — |

Cross-checked three ways: `bonicOS-firmware`'s `bonicbot_actuator_naming.md`
(BLE id registry), each series' `controllers.yaml`, and `robot_app`'s
`ROBOT_CONFIG`. And fitment is ultimately **per robot**: the ESP loads
`SERVO_CONFIGURED[]` and a `servo_config` limits blob from NVS at boot, so two
robots of the same series can differ.

So `servo_command` distinguishes three ways a joint can fail to move, and a
client needs all three:

- **`unknown`** — not a registry joint at all (a typo).
- **`unsupported`** — a real registry joint this robot does not fit. Dropped,
  not an error: the message is built from the controller's own joint list.
  Reported so a client can tell "absent" from "broken".
- **`failed`** — `{group: reason}`, most often "no known position yet" for a
  `Float64MultiArray` group. That message type carries no joint names — array
  position *is* joint identity — so the robot fills unspecified joints from the
  last `joint_states` sample and **refuses rather than guessing `0.0`**, since
  an unrequested joint snapping to zero is a real hazard on hardware.

The SDK never holds a series→joints table (see §3.1). It scopes commands and
convergence waits to the joints the robot **reports in `joint_states`**, which
is the one honest answer to "what does this robot have".

**Travel limits** are enforced by the URDF and independently by the ESP, and
they differ per series too — the gripper travels −45°..60° on A/S and
−60°..60° on M. The SDK does not validate user angles against these, but its
canned poses (`open_grippers`, `look_left`, …) command values valid on every
series, so they don't get clamped into a convergence timeout.

### 5.5 Head expression & LED matrix — ✅ live on A series

The server packs the `CMD_MATRIX_ACTION` body and publishes it on the series'
face-matrix topic; the ros2_control plugin forwards those bytes to the ESP
unchanged. A series with no such topic answers `ok:false` with
`no LED matrix on series <X>` — never a silent success.

| type | status | fields |
|---|---|---|
| `head_mode` | ✅ | `mode` (`normal`/`happy`/`sad`/`angry`/`surprised`/`confused`) |
| `head_look` | ✅ | `pan?`, `tilt?` (**radians**), `duration?`; `speed?` accepted and ignored |
| `display_text` | ✅ | `text` (ASCII) |
| `display_color` | ✅ | `r`, `g`, `b` (0-255) |
| `display_animation` | ✅ | `mode` — a name, `"play"`/`"pause"`, or a raw firmware index |
| `display_brightness` | ✅ | `value` (0-255) |
| `display_clear` | ✅ | — |

`head_mode` acks carry `substituted` when the requested expression has no face
in firmware and an approximation went out instead (`surprised` -> a heart,
`confused` -> a colour effect). Clients must surface it rather than treat the
call as an exact success.

`head_look` goes through `servo_command`'s head controller group, as this
section always anticipated. It carries radians like every other joint command
— the SDK converts from its own degrees boundary — and reports axes the robot
does not fit in `unsupported`, the same way `servo_command` does. A2 fits neck
yaw but no neck pitch, so `tilt` comes back unsupported there.

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
| `health` | ✅ live | — | `ack {type:"health", cpu_percent, ram_percent, disk_percent, temps:{...}, runcode?}` |
| `restart_base_session` | ✅ live | — | `ack {ok, error?, running, transitioning}` |
| `start_base_session` | ✅ live | — | `ack {ok, error?, running, transitioning}` |
| `stop_base_session` | ✅ live | — | `ack {ok, error?, running, transitioning}` |
| `get_session_status` | ✅ live | — | `ack {base:{...}, nav:{...}, health:{...}}` |
| `reconfig_wifi` | ✅ live | `ssid`, `password` | `ack {ok}` |
| `update_status` | ✅ live | — | `ack {ok, state, phase, percent, version, reported_version, previous_version, os_image_version, last_update, container}` |
| `shutdown` | ✅ live | — | `ack {ok, error?}` |
| `subscribe` | ✅ live | `events:[...]` (omit/empty ⇒ all) | `ack {ok, events:[...]}` |
| `set_scan_enabled` | ✅ live | `enabled` | `ack {ok, enabled}` |
| `set_camera_enabled` | ✅ live | `enabled`, `camera?` | `ack {ok, enabled, cameras}` |
| `get_camera_frame` | ✅ live | `camera?`, `since_seq?` | `ack {ok, camera, seq, encoding, data}` \| `ack {ok, camera, seq, unchanged}` |

`subscribe` narrows the telemetry firehose per client and replays cached
`map`/`costmap` for newly-covered events.

**`health` reports percentages, and no longer reports the container.**
`cpu_percent`/`ram_percent`/`disk_percent` are 0-100 numbers and `temps` is a
`{sensor: °C}` map — not the `cpu`/`ram`/`temp` this table used to name.
Container state left `health` when robot_app stopped updating itself: a
container asking whether it is running answers a question nobody had, and the
host owns that state. Ask `update_status` for it instead. `runcode` appears
only where a code runner is wired up, and carries its version so a dashboard
can spot a runner/robot_app mismatch.

**`shutdown` halts the machine, not the stack.** It powers the companion
computer down and, where the ESP lane is reachable, cuts the power latch, so
the robot ends up genuinely off rather than halted but still drawing current.
The ack is the whole reply — the process sending it is the one going away —
and it is idempotent: a second request while one is in flight is answered `ok`
rather than starting a second poweroff. For "restart the ROS stack", which is
what an operator usually wants, see `restart_base_session` above.

**Updates are answered by `update_progress`, not triggered here.** Installing
a version is a managed operation owned by bonic-host, outside the wire
protocol this SDK speaks — the SDK only reads what's happening. `state` is one
of `installing`, `rolling_back`, `idle`, `unavailable` (no bonic-host: a
bare-metal robot or a dev laptop), and `percent` is populated only while
`phase` is `pulling`, which is the only part whose progress is actually known.

Progress **stops mid-install by design**. The process reporting it is the one
docker replaces, so a client sees the pull, then its connection drop. The
health gate and any rollback are read afterwards from `last_update` — replayed
automatically on reconnect, or asked for with `update_status`.

**`start_base_session` / `stop_base_session` matter more than they look.**
`robot_app`'s `base_autostart` now defaults to **false on real hardware** —
bringing the app up (a deploy, a power cycle, a `systemctl restart`) must not
spin motors and energise servos on its own. A robot that booted with no stack,
or whose stack was stopped from here, has no other way back without SSH.
`stop_base_session` carries the same guard as `restart_base_session` and is
refused while the robot is moving or a goal is running.

**`set_scan_enabled` / `set_camera_enabled` are cost control, not capability.**
Neither changes what the robot can do; they change what it spends CPU on while
nobody is looking. Scan is the only telemetry topic not subscribed at startup
(10 Hz × ~1000 ranges, each frame costing a TF lookup and a downsample); the
ROS subscription is global (one ROS graph) but the *request* is per client and
reconciled across all of them, so it also stops when the last client that
wanted it disconnects without turning it off. `set_camera_enabled` idles one
viewer's WebRTC track — measured at ~32% of a core on a real A2 when unwatched
— without dropping it, since every signaling lane here is one-shot
offer/answer and removing a sender would need renegotiation. A lane with no
media tracks (local WS, BLE) answers `ok: false` rather than pretending.

**`get_camera_frame` is the video path for a client that cannot hold a media
track.** Video normally leaves the robot as WebRTC, and for a viewer that is
the right shape. The exception is the on-robot `run_code` runner: its sandbox
is `bwrap --unshare-net`, a network namespace containing nothing but its own
loopback, which is *not* the host's. A WebRTC peer started there has no route
to robot_app for signaling **or** media, and no ICE candidate pair between the
two namespaces can ever connect — so moving signaling onto the unix socket
alone buys an ICE timeout in place of a connection-refused. The socket is the
only thing that crosses the boundary, so the frame comes back in-protocol.

It is also the cheaper path for a frame-at-a-time caller. The bridge is
already holding the camera's JPEG undecoded, so this forwards bytes that
exist; the WebRTC route would decode that JPEG, re-encode it to VP8, and have
the client decode it again. Nothing about the media path changes — both run
side by side off the same latest-frame buffer, and a browser watching the
stream while a script polls frames costs no more than the browser alone.

`seq` counts frames stored for that camera since robot_app started, so a
caller quoting `since_seq` is answered `unchanged` with no payload when its
frame is still current — which is most calls, since a vision loop polls
faster than a camera publishes. It is per camera and resets on restart; a
client sees the reset as "different frame" and re-fetches, the safe
direction. `seq: 0` with `data: null` means the camera is configured but has
published nothing yet (a camera node still coming up, or one that died) —
reported rather than treated as an error, and distinct from `unchanged`,
which would tell a caller to keep showing a frame it never received. Exempt
from the rate limiter for the reason `drive` is: a vision loop runs at camera
rate, and it self-throttles anyway, being request/response with the caller
blocked on the ack.

**`llm_query` removed (2026-09-04).** The on-device LLM command (Ollama-backed
token streaming, `prompt`/`model?` → `llm_token` events, display-only) was
implemented but never actually used by any client, so both the SDK's
`ask_llm()`/`robot.ask_llm()` and robot_app's `llm_query` handler have been
deleted, along with the `llm_ondevice` feature flag that gated it. Not
implemented, not planned — if on-device LLM access comes back later it'll be
a new design, not a revival of this one.

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
| `scan` | `origin:{x,y,theta}, angle_min, angle_increment, range_min, range_max, ranges:[float\|null]` | `sensors.get_scan()`, `get_scan_points()` (**on demand** — see `set_scan_enabled`) |
| `nav_status` | `status: idle\|navigating\|succeeded\|failed\|canceled`, `goal_id?`, `distance_to_goal?` | `wait_for_goal()`, `get_nav_status()` |
| `dock_status` | `status: navigating\|succeeded\|failed\|canceled`, `goal_id`, `error?`, `error_code?` | `wait_for_dock()`, `get_dock_status()`, `get_dock_result()` |
| `nav_mode` | `mode: idle\|mapping\|navigating`, `map`, `transitioning`, `localized` | `get_nav_mode()` (cached, replayed on auth — same mechanism as `map`/`costmap`) |
| `base_session` | `running, owned, transitioning, error` | `system.get_base_session()` (cached, replayed on auth) |
| `session_health` | `ok, base:{...}, nav:{...}, issues:[...]` | `system.get_session_health()` (cached, replayed on auth; pushed only on change) |
| `update_progress` | `state, phase, percent, version, message, reported_version, last_update:{...}` | `system.get_update_status()` (cached, replayed on auth; pushed only while an install runs — see §5.7) |

**`scan` is unlike every other telemetry event, in two ways.** It is
subscribed **on demand only** — nothing arrives until a client sends
`set_scan_enabled` (§5.7), so a `None` from `get_scan()` may just mean nobody
asked. And it is emitted only while the robot is **localized**: the payload is
expressed in the map frame, so without a map→scanner transform there is no
correct place to draw the points, and drawing them at the origin would be worse
than drawing nothing. `ranges` is downsampled, and `angle_increment` is the
*effective* step after downsampling so a client places point *i* without
knowing the stride; `null` entries are inf/NaN "no return" readings.

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

For every 🔌 stub command, a server ships a handler now that returns a
**successful-shaped** `ack` (e.g. `{ "ok": true }`, empty lists for list-style
commands) and does nothing else, so client code runs end-to-end.

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

> **The 🔌 markers are the only record.** Since capability is never advertised
> (§3.1), this document and [`API.md`](./API.md) are the sole place a stub is
> distinguishable from a working command — someone watching a blank LED matrix
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

Vision pipelines (face/pose/object detection), autonomous exploration and
recorded sequences are **not** part of the v1 wire protocol. They are listed
here only so nobody assumes they were forgotten.
