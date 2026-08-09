# bonicOS-SDK (`bonicos`)

**One Python SDK for every BonicBot.** Replaces `bonicbot` (BLE + Android-app
WebSocket) and `bonicbot-bridge` (ROS via rosbridge) with a single package that
talks **one JSON protocol over one WebSocket** to the unified server in
`bonicOS-robot-app`.

> **Status:** v1 implemented — see `bonicos/` (package) and `tests/`
> (`pytest`/`mypy`/`flake8` all clean). These documents remain the contract
> the code satisfies; two spots where the docs left the exact byte-level
> contract to "whoever hosts it" are called out in `transports/webrtc.py`'s
> module docstring (the SharedArrayBuffer layout) and `discovery.py`'s (the
> mDNS service type) — both are this build's proposed convention, pending
> alignment with the bonic.ai front end / robot-side teams.

---

## Why this exists

Today there are two Python SDKs with two transports and two overlapping-but-
different command surfaces:

| SDK | Transport | Surface |
|---|---|---|
| `bonicbot` | BLE binary + WS to the Android app | servo-by-id, head expression modes, hand gestures, LED matrix, base motors, battery/distance; speak/sequences/camera via the app |
| `bonicbot-bridge` | roslibpy over rosbridge `:9090` | motion + precise motion, Nav2 + locations + exploration, arm/gripper/neck controllers, vision pipeline, camera, sensors |

A student's code is not portable between them, hardware primitives and ROS
primitives live in different libraries, and neither speaks the transport the new
`bonicOS-robot-app` server exposes. `bonicos` collapses both into **one wheel,
one API, one protocol**, transport-agnostic underneath.

## What `bonicos` is

- A **synchronous, blocking, student-friendly** Python API (the reactor/thread
  machinery is hidden), matching the ergonomics both old SDKs already had.
- A **transport-agnostic** client. The same `robot.py` logic runs over:
  - **WebSocket** — native Python (developer laptop, on-robot runner container).
  - **WebRTC + SharedArrayBuffer** — inside the browser (Pyodide, the bonic.ai
    developer studio). *Pro models only.*
  - **sim** / **mock** — Three.js digital twin and hardware-free tests.
- **Host-agnostic.** It connects to a WebSocket endpoint speaking the `bonicos`
  protocol and does not care *who* serves it (see topology below).

## The one thing that surprises people: who hosts the server

`bonicos` always connects to a WebSocket server. **Which process is that server
depends on the robot model.**

| Model | Has a processor (Pi/Jetson)? | WS server host | Browser (Pyodide) coding? |
|---|---|---|---|
| **Lite** (s1-lite, a2-lite) — always ships with an Android tablet | No | **The Flutter app** on the tablet | No (native SDK only) |
| **Pro** with Android tablet | Yes | `bonicOS-robot-app` on the processor | Yes |
| **Pro** without tablet (a2-pro) | Yes | `bonicOS-robot-app` on the processor | Yes |

The SDK is identical in all three cases. The protocol is identical. Only the
process on the other end of the socket differs. This is the whole reason the
protocol contract ([PROTOCOL.md](./PROTOCOL.md)) is defined independently of the
server.

## Read these in order

1. **[ARCHITECTURE.md](./ARCHITECTURE.md)** — package layout, transports, the
   sync facade, the Pyodide/SharedArrayBuffer bridge, model topology, and the
   design decisions (and the reasoning behind each).
2. **[PROTOCOL.md](./PROTOCOL.md)** — the wire contract: message envelope,
   every command, telemetry events, versioning, the stub convention, and
   `speak` routing. Shared verbatim with `bonicOS-robot-app`.
3. **[API.md](./API.md)** — the user-facing Python API: classes, every method,
   signatures, blocking semantics, and worked examples.

## Scope of the first release (v1)

**Core first: motion, navigation, servos/arms, telemetry.** Deliberately
included even where the robot-side ROS topic does not exist yet — those land as
**stub handlers** on the server (log + no-op) so the SDK API is complete and
stable before `bonicOS-m1-ros` grows the topics.

**Deferred to a later phase:** vision pipeline (YOLO/face/pose/gesture/ArUco),
autonomous exploration, sequences. Live camera-frame streaming is implemented
on native and **SDK-side-only** on the browser transport (`webrtc.py`'s
`frames` SharedArrayBuffer contract is fully specified and read; see
ARCHITECTURE.md §3.2) — end-to-end use still needs the bonic.ai front end to
build the host half (Pyodide worker + peer connection + writing decoded
frames into that buffer), which doesn't exist yet.

**Not carried over:** BLE transport (everything routes through the server;
BLE is demoted to provisioning per the platform architecture) and any
backward-compatibility shim for `bonicbot` / `bonicbot-bridge` (no significant
user base — clean break).

## Related documents (other repos)

- `bonicOS-robot-app/bonic-architecture.md` — platform architecture (browser
  execution, transports, SAB bridge, safety, access control).
- `bonicOS-robot-app/bonicbot_webrtc_protocol_spec.md` — WebRTC transport +
  local WS lane wire format; §7/§9 already name `bonicos` as this repo.
- `bonicOS-robot-app/OVERVIEW.md` — the server this SDK talks to.
