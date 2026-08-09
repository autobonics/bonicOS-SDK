# bonicos — SDK Architecture

How the package is put together and why. Read [PROTOCOL.md](./PROTOCOL.md) for
the wire format and [API.md](./API.md) for the public surface.

---

## 1. Package layout

**One PyPI wheel. Multiple transports. Identical public API everywhere.** This
mirrors `bonic-architecture.md` §4.

```
bonicos/
  __init__.py           # exports BonicBot + enums + exceptions; nothing heavy imported here
  robot.py              # BonicBot — all user-facing logic, transport-agnostic
  protocol.py           # message schema constants + PROTOCOL_VERSION (mirrors PROTOCOL.md)
  exceptions.py         # RobotError, ConnectionError, CommandError, RobotDisconnected, FeatureUnavailable, ...
  enums.py              # ServoID, HeadMode, etc. (carried from the old SDKs, trimmed to core)

  controllers/          # feature groups delegated to by BonicBot (see API.md)
    motion.py           # drive, move_forward/backward, turn, stop
    precise_motion.py   # drive_distance / rotate_angle / drive_and_rotate / draw_square / queue  (CLIENT-SIDE loop, v1)
    navigation.py       # nav_goal, waypoints, locations, mapping, initial pose, nav status
    arm.py              # arm/gripper/neck via servo_command groups
    head.py             # head pan/tilt + expression modes + LED matrix (server stubs in v1)
    sensors.py          # telemetry reads: position, battery, imu, joint states
    system.py           # health, speak, wifi, update, feature flags

  transports/
    base.py             # Transport ABC (the interface robot.py depends on)
    websocket.py        # native — lazy-imports `websockets`; background rx thread
    webrtc.py           # pyodide — lazy-imports `js`, `pyodide.ffi`; SharedArrayBuffer bridge
    sim.py              # drives the Three.js digital twin through the same buffers
    mock.py             # no hardware, deterministic, for tests

  discovery.py          # optional mDNS lookup of a robot by id (native)
```

**Lazy-import rule (hard requirement).** `websockets` does not exist in Pyodide;
`js` / `pyodide.ffi` do not exist natively. **Neither may be imported at package
load time.** `transports/websocket.py` and `transports/webrtc.py` import their
backend inside `connect()`, never at module top. `bonicos/__init__.py` must not
import either transport eagerly.

---

## 2. The transport interface

`robot.py` never mentions WebSocket, WebRTC, or SharedArrayBuffer. It depends
only on this interface (`transports/base.py`). Every backend implements it.

```python
class Transport(Protocol):
    def connect(self, timeout: float) -> dict: ...
        # performs the handshake; returns the auth_result payload
        # (robot_id, series, features) so BonicBot can expose feature flags.

    def send(self, msg: dict) -> int: ...
        # enqueue a command; returns a monotonic client command id used to
        # correlate the ack.

    def read_telemetry(self) -> dict: ...
        # latest merged telemetry snapshot, non-blocking. Native: last value
        # per event under the GIL. Pyodide: seqlock read from the SAB.

    def wait_for_update(self, timeout: float) -> bool: ...
        # block until the next telemetry frame; paces loops to real sensor rate.

    def wait_for_ack(self, cmd_id: int, timeout: float) -> dict: ...
        # block until the server acks/errors this command id.

    def read_frame(self) -> "np.ndarray | None": ...
        # latest camera frame (BGR) or None. Live on native (own aiortc peer)
        # and SDK-side-implemented on Pyodide/WebRTC (reads the `frames` SAB
        # per §3.2) — the browser HOST HALF (writing decoded frames into that
        # buffer) doesn't exist in bonic.ai yet, so it's untestable
        # end-to-end until that lands.

    def close(self) -> None: ...
```

This interface is the single seam. Everything below diverges per environment;
everything above is written once.

---

## 3. Two runtimes, two concurrency models

### 3.1 Native (WebSocket) — background rx thread

Plain Python has real threads, so the SAB machinery from the browser is
unnecessary. Per `bonic-architecture.md` §11, native collapses to a background
receive thread that rebinds the latest telemetry under the GIL:

```python
# transports/websocket.py (sketch — NOT final code, illustrates the model)
def _rx_loop(self):
    for raw in self._ws:                 # blocking iteration on the socket
        msg = json.loads(raw)
        t = msg.get("type")
        if t in TELEMETRY_EVENTS:
            self._latest[t] = msg        # atomic rebind under the GIL
            self._update_event.set(); self._update_event.clear()
        elif t in ("ack", "error"):
            self._acks[msg["id"]] = msg  # unblock wait_for_ack
        elif t in ASYNC_EVENTS:          # nav_status, llm_token, run_code_output...
            self._events.append(msg)

def read_telemetry(self):   return dict(self._latest)
def wait_for_update(self, t=1.0): return self._update_event.wait(t)
```

The **public API stays synchronous** — `robot.move_forward()` returns when the
work is acknowledged/complete. Students never touch `async`/`await`. Both old
SDKs already worked this way (bonicbot spun an asyncio loop in a thread; bridge
used roslibpy's reactor), so this is the proven ergonomic.

### 3.2 Pyodide (WebRTC) — SharedArrayBuffer bridge

Inside the browser the constraints from `bonic-architecture.md` §5 apply and the
SDK **must not reimplement them** — it consumes buffers a host preloader set up:

- `RTCPeerConnection` is main-thread only; Pyodide runs in a Web Worker; so the
  peer connection and Python live on different threads.
- The bridge is **SharedArrayBuffer, not `postMessage`** — a student's
  `while True:` loop never yields, so a postMessage telemetry path would freeze.
- Telemetry is read via a **seqlock** (never returns a torn value). Blocking uses
  `Atomics.wait`, which is legal in a worker.
- Command ring buffer (worker writes, main thread drains); completion slots
  (main writes, worker waits); one fixed-size seqlock slot per camera for
  frames (host writes, worker reads — same seqlock discipline as telemetry).

The SDK's `transports/webrtc.py` is the **worker/Python side** of that bridge:
read the seqlock, write to the command ring, `Atomics.wait` on completion slots,
read camera frames out of their slots. It assumes the host (bonic.ai front end)
created and connected the peer + SABs and handed them in. See §5 for the
constructor's role here.

> Buffer byte layout and seqlock write/read for telemetry/commands/completions
> are specified in `bonic-architecture.md` §5–6 and are the **host/front-end's**
> responsibility to produce; the SDK only reads/writes them. Keep them in
> exactly one place. **Frames are the exception**, fully specified by this SDK
> (`webrtc.py`'s module docstring — one RGBA8 seqlock slot per camera,
> `frames.byteLength // len(cameras)` bytes each, slot *i* matching
> `auth_result["cameras"][i]`) since no other document had claimed that byte
> layout; align `bonic-architecture.md` to it rather than inventing a second
> one. Not yet produced by any host build — see README "Scope of the first
> release".

---

## 4. Where control loops live (v1 decision)

**Precise motion (`drive_distance`, `rotate_angle`, `drive_and_rotate`,
`draw_square`) and command queues run CLIENT-SIDE in `controllers/precise_motion.py`
for v1.** They are built on `drive` + odom telemetry:

```
loop: read odom → compute error → send drive() → wait_for_update() → repeat → send stop()
```

- Rationale: this is how `bonicbot-bridge` works today (its 847-line
  `precisemotion.py`); porting it as-is gets a complete SDK out fastest.
- **Safety backstop:** the on-Pi cmd_vel deadman (`bonicOS-robot-app`
  `core/deadman.py`) publishes zero if no `drive` frame arrives for 400 ms, so a
  stalled client loop or dropped link stops the robot regardless.
- **Known divergence from `bonic-architecture.md` §3** ("real-time loops belong
  on the Pi, network-independent"). Accepted for v1. The migration path is a
  clean swap: replace the client loop with a single `drive_distance` *intent*
  command once `bonicOS-m1-ros` exposes an on-Pi motion server; the public API
  signature does not change. Design `precise_motion.py` so the loop body is
  isolated behind the same method the future intent will call.

Everything that is *not* a tight loop (Nav2 goals, mapping) already runs on the
Pi and is a fire-and-monitor command.

### 4a. Completion detection — evidence from sim testing (2026-08-04)

**Rule: never decide "has this command finished" from a fixed sleep keyed to
a commanded duration. Poll telemetry until the target is reached, with a
generously padded timeout as the only backstop.** This was proven, not just
asserted — a servo test harness driving `robot_app` over WS showed spurious,
non-reproducible "failures" (different joints, different runs) as long as it
used `time.sleep(duration)` to decide when to check. Switching the exact same
test to poll `/joint_states` on the ROS/sim clock instead of wall-clock time
made every run pass cleanly (18/18 joints, 3/3 runs). The failures were 100%
a test-timing artifact — `robot_app`'s `servo_command` handler was correct
the whole time — but the artifact is real and will hit any SDK code that
makes the same assumption, especially against the `sim` transport where
Gazebo's RTF is not exactly 1.0.

This is exactly why `precise_motion.py`'s `drive_distance`/`rotate_angle`
poll odometry in a loop instead of sleeping for an estimated duration — that
design was already correct. **The same pattern has been extended to servo
commands** (`controllers/arm.py`, implemented 2026-08-04):

- `move_left_arm(..., wait=True)` (and `set_servos`, `move_right_arm`)
  previously called `_command()` and returned as soon as it got an
  **ack** — i.e. `wait=True` meant "the server acknowledged the command,"
  not "the arm physically reached the target." The ack arrives almost
  immediately, before the trajectory ramp even starts.
- **Fixed:** after a successful ack, `_send_servo_command` now calls
  `_wait_for_convergence`, which polls `get_servo_angles()` (itself fixed —
  see below) until every commanded joint is within `CONVERGENCE_TOLERANCE_DEG`
  (8.6°, the exact `0.15` rad criterion `multiTestReport.md` §4 vetted
  end-to-end) of its target, or a padded timeout elapses. Timeout defaults
  to `max(duration * 3, 5.0)` when not given explicitly — padded well above
  the nominal `duration`, per the rule above — and is now an exposed
  `timeout=` parameter on `set_servos`/`move_left_arm`/`move_right_arm`.
  Any key the server reported as `unknown` (PROTOCOL.md §5.4) is excluded
  from the convergence check, so a typo'd joint name can't make the whole
  call spuriously time out despite every valid joint arriving fine.
- **A dependent bug this surfaced and also fixed:** `get_servo_angles()`
  previously returned `joint_states` telemetry keyed by whatever names
  arrived on the wire — but the server reports **snake_case URDF joint
  names** (`left_elbow_joint`), never the **camelCase registry keys**
  (`leftElbow`) a command is sent with. So the old implementation's return
  value used a different key convention than its own input methods, and a
  convergence check built on it wouldn't have matched anything even if it
  existed. Fixed by adding `protocol.JOINT_NAME_MAP` (the exact 18-entry
  camelCase↔snake_case table, mirroring `bonicOS-robot-app/app/config.py`'s
  `ACTUATOR_JOINTS`, verified round-trip end-to-end against the real M1 sim)
  and translating through it in both directions.
- **Also fixed while touching this:** `enums.py`'s `ServoID` was the old
  BLE-only hardware's joint set (ported from `Bonicbot-SDKs/bonicbot`), not
  the M1's — it named three joints that don't exist on M1 at all
  (`rightWrist`, `headPan`, `headTilt`) and was missing six real ones
  (per-axis wrist pitch/yaw, gripper yaw, `neckPitch`). `reset_servos()`
  built its payload from `{joint.value for joint in ServoID}`, so it was
  silently sending nonsense keys and failing to reset 6 real joints. Now
  every `ServoID` member is a valid key in `protocol.JOINT_NAME_MAP` — all
  18, no more, no less.
- `servo_command` is **not** in `router.py`'s `HIGH_RATE_COMMANDS` exemption
  (unlike `drive`) — fine for discrete servo moves, but a future
  continuous/streamed servo-control feature would need the same rate-limit
  exemption `drive` already got.

**Trajectory preemption (verified, informational — no SDK change needed):**
sending a new `servo_command` to a controller group that's still executing a
previous trajectory **immediately preempts** it — the joint interpolates
smoothly from its *current actual position* (not from home, not from the
old target) to the new target over the new command's duration. No jerk, no
queueing, no rejection. Confirmed by driving a 3.0s trajectory, interrupting
it 1.0s in with a different 1.0s-duration target, and sampling position
continuously: the joint never continued toward the old target after the
interrupt and converged cleanly on the new one. Consequence for the SDK: two
servo/arm calls issued back-to-back without waiting for the first to
complete is safe and well-defined — the second simply takes over — which is
almost certainly the desired semantics for teleoperation-style code, but
worth stating explicitly since nothing was queuing.

### 4b. Partial-joint servo commands are a silent no-op — `_fill_group` (2026-08-04)

**A more serious bug than 4a's, found chasing what first looked like renewed
flakiness after 4a's fixes shipped:** `move_left_arm(shoulder, elbow)`,
`set_neck(yaw)`, and any `set_servos({...})` call naming fewer than a full
controller group's joints **did nothing** — acked `ok: true`, but the arm
never moved — on `left_arm`/`right_arm` (`JointTrajectoryController`, 7
joints each) and `head` (`JointGroupPositionController`, 2 joints).

**How this was pinned down, not guessed:** direct `rclpy` wire-watching
showed the message *did* reach the controller topic with correct content,
but the controller's own internal `reference` (goal) state never updated —
the classic signature of `ros2_control` rejecting a trajectory that omits
one of the controller's claimed joints. Confirmed independently by an
external, rigorous ROS-level test — 60 rapid-fire iterations per joint,
`bonicOS-m1-ros/multiTestReport_stress.md` — whose own Guideline #2 states
this requirement explicitly: *"Always bundle complete 7-DOF vectors for arm
trajectories... never omit joints."* That report's controlled methodology
always sent full vectors, which is exactly why it never encountered the bug
this SDK had. It separately, independently confirmed 16/18 joints at 100%
(60/60) reliability and both `left_elbow_joint`/`right_elbow_joint` at 0/60
— a real, reproducible, sim-side (Gazebo `dynamics damping`/effort tuning)
issue unrelated to this SDK or `robot_app`, not something in scope here.

**Fixed:** `ArmController._fill_group()` runs before every `servo_command`
send. For each controller group touched by the caller's keys
(`protocol.JOINT_GROUPS`/`JOINT_GROUP_OF`), any group member the caller
*didn't* specify is filled in at its current measured position (from the
now-correct `get_servo_angles()`), falling back to `0.0` only if no
telemetry for it has ever arrived. So `move_left_arm(shoulder, elbow)` now
sends all 7 `left_arm` joints — the 2 requested plus the other 5 held where
they already are — which is what the controller actually requires. Verified
live post-fix: `set_servos({"leftShoulderRoll": 45})` (a 1-of-7 partial
call) reached 43.6° against a 45° target; `set_neck(25)` (1-of-2) reached
16.6° — both real, measured motion, not the previous stuck-at-0.

Single-joint groups (`left_gripper`, `right_gripper`) are unaffected by
definition — nothing else in a 1-joint group to fill. Keys outside any known
group (typos) pass through unchanged for the server's normal `unknown`
handling (§4a).

---

## 5. Constructor & connection model

The constructor **must be identical across environments** so student code is
portable between the platform and a robot at home (`bonic-architecture.md` §4).

```python
BonicBot()                                   # pyodide: bind the preloaded singleton transport
                                             # native:  mDNS autodiscovery (discovery.py)
BonicBot("192.168.1.50", robot_id="M1_001")  # explicit host (developer laptop → robot / tablet)
BonicBot(host="127.0.0.1", robot_id="M1_001")# on-robot runner container (localhost)
```

Environment detection (native vs Pyodide) is automatic: check for the `pyodide`
module / a host-provided global. The chosen transport follows from that, never
from a constructor argument the student has to set.

**`robot_id` and the WS endpoint.** Native connects to
`ws://<host>:8080/ws?robotId=<robot_id>`. The server closes with `4404` on a
robotId mismatch (a wrong-robot guard, not security — see PROTOCOL.md §Auth).
`robot_id` is therefore required natively unless discovery supplies it.

**Host-agnostic across models (see README topology).** On **pro** models `host`
is the processor running `bonicOS-robot-app`; on **lite** models `host` is the
tablet running the Flutter WS server. The SDK code path is identical — same URL
shape, same protocol, same handshake. Pyodide/browser coding is **pro-only**;
the SDK never has to special-case lite beyond "someone else hosts the socket."

**Token.** If a `BONICOS_TOKEN` env var is present the handshake includes it.
It is ignored by the server in v0 (access is gated platform-side) but sent for
forward-compatibility with v1 proximity-gated auth. Never a required argument.

---

## 6. Feature gating

The handshake returns the robot's `features` map (from the server's `config.py`
series matrix). `BonicBot` exposes it (e.g. `robot.features`) and each controller
checks it before sending a gated command, raising `FeatureUnavailable` with a
clear message rather than sending a command the robot will reject. The server
*also* gates (returns `feature_unavailable`) — the SDK check is for a fast, local,
readable failure, not the security boundary.

---

## 7. Simulation & mock parity

- **sim**: `transports/sim.py` reads/writes the *same* buffers the Three.js
  digital twin publishes into (`bonic-architecture.md` §10). `robot.py` cannot
  tell sim from real — zero branching in the user-facing layer.
- **mock**: `transports/mock.py` is a deterministic, hardware-free double for
  unit tests (telemetry you can script, acks you can assert on). This is the
  test seam; no network, no ROS, no browser.

---

## 8. Packaging

- Single wheel `bonicos`, Python 3.11+.
- Base deps minimal. `websockets` is an **optional/native extra** (absent in
  Pyodide) — lazy-imported so a Pyodide install never needs it. `numpy` only
  where frames are actually used — the `[camera]` extra on native
  (`aiortc`+`numpy`, lazy-imported in `_camera_link.py`); a Pyodide package
  the host must preload on the browser side, never installed via pip there.
- `pyproject.toml`, semantic version; `PROTOCOL_VERSION` lives in
  `protocol.py` and is sent in the handshake (see PROTOCOL.md §Versioning).
- Ship `py.typed` — the API is fully type-hinted.

---

## 9. Design decisions (summary)

| Decision | Choice | Why |
|---|---|---|
| Number of transports in the API | 1 (invisible) | student code is portable; env picks the backend |
| BLE | dropped from the SDK | everything routes through the server; BLE is provisioning-only per platform arch |
| Concurrency exposed to users | none — sync facade | education audience; matches both old SDKs |
| Precise-motion loop | client-side (v1) | ships fastest; deadman backstops; clean migration later |
| Backward-compat shims | none | negligible existing user base — clean API |
| Vision / exploration / sequences | deferred | core-first release |
| Camera frames on Pyodide/WebRTC | SDK-side implemented, host half not yet built | `webrtc.py` reads a fully-specified `frames` SAB; native already worked. No frontend Pyodide/SAB scaffolding exists yet to write into it (verified 2026-08-09 — not even a skeleton) |
| Who owns SAB byte layout | the browser host, not the SDK, for telemetry/commands/completions; the SDK itself, for frames (no other doc had claimed it) | one source of truth per buffer (`bonic-architecture.md` §5 for the former, `webrtc.py`'s module docstring for the latter) |
| Completion detection | poll telemetry, never sleep-on-duration | proven with real flaky-vs-clean test evidence (§4a); implemented in `arm.py`'s `wait=True` |
| Servo trajectory preemption | new command replaces old immediately, no queue | verified (§4a); back-to-back calls are safe by design |
| `list_maps()` return shape | extract `name` from server metadata dicts | server returns `{name,size,modified}` dicts, not plain strings — verified against real M1 sim 2026-08-04; fixed to honor the documented `List[str]` contract |
| Partial servo commands | always send the full controller-group vector (`_fill_group`) | a partial trajectory is a silent no-op on `left_arm`/`right_arm`/`head` — verified live + cross-checked against an independent ROS-level stress test (§4b) |
