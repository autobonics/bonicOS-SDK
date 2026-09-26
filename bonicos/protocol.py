"""Wire schema constants — the machine-readable mirror of ``PROTOCOL.md``.

Pure data, no I/O. Safe to import from every transport at module load time.
"""

from __future__ import annotations

#: Bumped on any breaking change to a command/event shape in PROTOCOL.md.
#: Additive commands/fields do not bump it. Sent as ``protocol_version`` in
#: the ``auth`` handshake message (PROTOCOL.md §3, §9).
PROTOCOL_VERSION = 1

#: Envelope / response type strings (PROTOCOL.md §2).
TYPE_AUTH = "auth"
TYPE_AUTH_RESULT = "auth_result"
TYPE_ACK = "ack"
TYPE_ERROR = "error"

# --- Commands (client -> robot), grouped per PROTOCOL.md §5 -----------------

#: §5.1 Motion — high-rate, no id, no ack.
CMD_DRIVE = "drive"

#: §5.2 Navigation & mapping.
CMD_NAV_GOAL = "nav_goal"
CMD_NAVIGATE_THROUGH_WAYPOINTS = "navigate_through_waypoints"
CMD_CANCEL_NAV = "cancel_nav"
CMD_SET_INITIAL_POSE = "set_initial_pose"
CMD_START_NAVIGATION = "start_navigation"
CMD_STOP_NAVIGATION = "stop_navigation"

#: Runtime nav-stack session switching (mapping ⇄ navigation), owned by
#: robot_app's NavModeManager — distinct from start_mapping/stop_mapping
#: below, which only pause/unpause slam_toolbox *within* an already-entered
#: mapping session. enter_navigation_mode launches map_server+AMCL+Nav2 on a
#: saved map; enter_mapping_mode launches slam_toolbox+Nav2 from scratch.
#: Both tear down whichever session is currently up first.
CMD_ENTER_MAPPING_MODE = "enter_mapping_mode"
CMD_ENTER_NAVIGATION_MODE = "enter_navigation_mode"
CMD_STOP_NAV_MODE = "stop_nav_mode"
CMD_GET_NAV_MODE = "get_nav_mode"

CMD_START_MAPPING = "start_mapping"
CMD_STOP_MAPPING = "stop_mapping"
CMD_SAVE_MAP = "save_map"
CMD_LOAD_MAP = "load_map"
CMD_DELETE_MAP = "delete_map"
CMD_LIST_MAPS = "list_maps"

#: Base ROS stack (drive/sensors/TF/controllers — the layer NavModeManager's
#: sessions run on top of) supervision, owned by robot_app's
#: BaseSessionManager. ``restart_base_session`` is refused while the robot is
#: moving or navigating (cancel/stop first); it is slow (a cold Gazebo start
#: is ~25s, on top of nav teardown/reseed) so the SDK uses a much longer
#: default timeout than other commands. ``get_session_status`` is a
#: synchronous point-in-time read of the base+nav+health state — the same
#: information the ``base_session``/``session_health`` telemetry events push
#: on change.
#:
#: ``start_base_session``/``stop_base_session`` are the separate halves, and
#: they matter more than they look: robot_app's ``base_autostart`` now
#: defaults to FALSE on real hardware (booting the app must not energise
#: servos on its own), so a robot that came up with no stack has no other way
#: in without SSH. ``stop_base_session`` carries the same moving/goal guard as
#: restart.
CMD_RESTART_BASE_SESSION = "restart_base_session"
CMD_START_BASE_SESSION = "start_base_session"
CMD_STOP_BASE_SESSION = "stop_base_session"
CMD_GET_SESSION_STATUS = "get_session_status"

#: §5.3 Named locations — LIVE since 2026-08-31 (robot_app
#: ``managers/location_store.py``), no longer stubs. A location is a
#: **map-frame pose**, so every one of these resolves a map first: an explicit
#: ``map`` field, else the map the running navigation session has loaded. The
#: two that involve the robot rather than just the store — ``goto_location``,
#: and ``save_location`` in its "save where I am" form — additionally require
#: that map to be the one Nav2 currently has open, because a pose from another
#: map is a well-formed coordinate pointing at a different room.
CMD_SAVE_LOCATION = "save_location"
CMD_GOTO_LOCATION = "goto_location"
CMD_DELETE_LOCATION = "delete_location"
CMD_DELETE_ALL_LOCATIONS = "delete_all_locations"
CMD_LIST_LOCATIONS = "list_locations"

#: §5.3.1 Docking — addon only. A dock pose is map-frame like a location but
#: stored separately, so it never appears in ``list_locations``.
#: ``save_dock`` has no x/y form: it records where the robot is parked. A
#: robot without the docking addon refuses ``save_dock``/``dock``/``undock``;
#: ``list_docks``/``delete_dock`` answer on any robot. ``dock``/``undock`` ack
#: ``{goal_id}`` once the goal is accepted and report on ``dock_status``.
CMD_SAVE_DOCK = "save_dock"
CMD_LIST_DOCKS = "list_docks"
CMD_DELETE_DOCK = "delete_dock"
CMD_DOCK = "dock"
CMD_UNDOCK = "undock"

#: §5.4 Servos / arms / grippers / neck.
CMD_SERVO_COMMAND = "servo_command"
CMD_SERVO_SINGLE = "servo_single"

#: §5.5 Head expression & LED matrix — all stub in v1.
CMD_HEAD_MODE = "head_mode"
CMD_HEAD_LOOK = "head_look"
CMD_DISPLAY_TEXT = "display_text"
CMD_DISPLAY_COLOR = "display_color"
CMD_DISPLAY_ANIMATION = "display_animation"
CMD_DISPLAY_BRIGHTNESS = "display_brightness"
CMD_DISPLAY_CLEAR = "display_clear"

#: §5.6 Speech.
CMD_SPEAK = "speak"
#: ``speak`` engines: an on-device voice (the default) or a cloud voice.
SPEAK_ENGINE_EDGE = "edge"
SPEAK_ENGINE_CLOUD = "cloud"
SPEAK_ENGINES = (SPEAK_ENGINE_EDGE, SPEAK_ENGINE_CLOUD)
#: ``speak`` rate bounds, inclusive. 1.0 is normal speed; higher is faster.
SPEAK_RATE_RANGE = (0.5, 2.0)

#: §5.7 System & session.
CMD_HEALTH = "health"
CMD_RECONFIG_WIFI = "reconfig_wifi"
#: Point-in-time read of the update state the robot otherwise pushes as
#: ``update_progress``. Needed because an install replaces the process that
#: was pushing it: the outcome is read back across the reconnect.
CMD_UPDATE_STATUS = "update_status"
#: Halt the companion computer, and cut the power latch where the ESP
#: lane can reach it — the robot ends up genuinely off, not halted-but-
#: powered. Nothing comes back but the ack: the process answering is the
#: one going away.
CMD_SHUTDOWN = "shutdown"
CMD_SUBSCRIBE = "subscribe"

#: Cost control for the two streams that are expensive to produce and usually
#: unwatched. Neither changes what the robot *can* do — they change what it
#: spends CPU on while nobody is looking, which is why they are commands
#: rather than a client-side filter.
#:
#: ``set_scan_enabled`` toggles the ONLY telemetry topic robot_app does not
#: subscribe at startup (10 Hz x ~1000 ranges, each frame needing a TF lookup
#: and a downsample). The ROS subscription is global — there is one ROS graph —
#: but the *request* is per client and reconciled across all of them, so it
#: also goes away when the last client that wanted it disconnects without
#: turning it off. ``{enabled}`` -> ``ack {ok, enabled}``.
#:
#: ``set_camera_enabled`` idles a WebRTC video track this viewer has hidden.
#: The robot attaches a track per camera at peer setup and cannot otherwise
#: know the client's camera panel is closed; unwatched, that stream measured
#: ~32% of a core on a real A2. Idling does not drop the track (every
#: signaling lane here is one-shot offer/answer, so removing a sender would
#: need renegotiation) — recv() just slows to a static frame a second, and
#: re-enabling is instant. Per client and WebRTC-only: a lane with no media
#: tracks answers ``ok: false`` rather than pretending.
#: ``{enabled, camera?}`` -> ``ack {ok, enabled, cameras}``.
#: ``get_camera_frame`` returns ONE frame as JPEG over the control lane
#: itself, instead of the WebRTC media track video normally rides. It exists
#: for the on-robot ``run_code`` runner, whose sandbox (``bwrap
#: --unshare-net``) has a network namespace containing only its own loopback:
#: a WebRTC peer there cannot reach robot_app for signaling *or* media, since
#: no ICE candidate pair between the two namespaces can ever connect. The
#: unix socket is the only path across, so the frame comes back in-protocol.
#: ``since_seq`` quotes the sequence number the caller already holds, so a
#: poll that outruns the camera is answered ``unchanged`` with no payload.
#: ``{camera?, since_seq?}`` ->
#: ``ack {ok, camera, seq, encoding: "jpeg", data: <base64>}``
#: | ``ack {ok, camera, seq, unchanged: true}``
#: | ``ack {ok, camera, seq: 0, data: null}`` (camera configured, nothing
#: published yet).
CMD_SET_SCAN_ENABLED = "set_scan_enabled"
CMD_SET_CAMERA_ENABLED = "set_camera_enabled"
CMD_GET_CAMERA_FRAME = "get_camera_frame"

#: Commands that are never acked (high-rate) — the SDK must not
#: `wait_for_ack` on these.
UNACKED_COMMANDS = frozenset({CMD_DRIVE})

# --- Capability: deliberately not modelled (PROTOCOL.md §3.1) ---------------
#
# There is no feature map here, and there must not be one. The handshake
# advertises identity only, the SDK sends whatever it is asked to, and a robot
# that cannot perform a command answers with an `error` whose message explains
# why. A server with no handler for a command cannot disagree with its own
# hardware the way a capability table can.

# --- Telemetry & async events (robot -> client), PROTOCOL.md §6 -------------

EVENT_POSE = "pose"
EVENT_ODOM = "odom"
EVENT_BATTERY = "battery"
EVENT_JOINT_STATES = "joint_states"
EVENT_IMU = "imu"
EVENT_MAP = "map"
EVENT_COSTMAP = "costmap"
EVENT_PLAN = "plan"
EVENT_NAV_STATUS = "nav_status"

#: A dock/undock attempt, reported like ``nav_status`` — same ``goal_id``,
#: same ``navigating`` → ``succeeded``/``failed``/``canceled`` vocabulary.
#: Adds ``error`` and ``error_code`` on a failed attempt.
EVENT_DOCK_STATUS = "dock_status"

#: Downsampled laser scan, already transformed into the **map** frame:
#: ``{"origin": {"x", "y", "theta"}, "angle_min", "angle_increment",
#: "range_min", "range_max", "ranges": [float | None, ...]}``. ``origin`` is
#: where the scanner is in the map, and ``angle_increment`` is the EFFECTIVE
#: step after downsampling, so a client places point *i* without knowing the
#: stride. ``None`` entries are inf/NaN "no return" readings (not
#: JSON-serialisable, and a caller has to skip them either way).
#:
#: Two things make this unlike every other telemetry event. It is subscribed
#: **on demand only** — see ``CMD_SET_SCAN_ENABLED``; nothing arrives until
#: someone asks. And it is emitted only while the robot is **localized**,
#: because the map-frame transform is what the payload is expressed in and
#: drawing these at the origin would be worse than drawing nothing.
EVENT_SCAN = "scan"

#: Current mapping/navigation session state, pushed on every transition by
#: NavModeManager (``{"mode": "idle"|"mapping"|"navigating", "map": str|None,
#: "transitioning": bool, "localized": bool}``). Cached-latest like
#: ``map``/``costmap``, not a stream — see ``CMD_GET_NAV_MODE`` for an
#: explicit synchronous read. ``localized`` reflects whether AMCL/slam_toolbox
#: currently owns a live map-frame pose (freshness-checked, not just latched
#: from the last seeding attempt) — a robot can be ``navigating`` and still
#: unlocalized if AMCL's seed hasn't landed yet or it later loses the pose.
EVENT_NAV_MODE = "nav_mode"

#: Base ROS stack (drive/sensors/TF/controllers) up/down state, pushed by
#: BaseSessionManager on every transition (``{"running": bool, "owned": bool,
#: "transitioning": bool, "error": str|None}``). ``owned`` is False for a
#: stack robot_app adopted rather than spawned itself (e.g. one already
#: running via start_session.sh) — restart/stop still work either way.
EVENT_BASE_SESSION = "base_session"

#: Combined "is this robot actually working" signal, pushed by
#: SessionHealthMonitor on change (not a heartbeat — a healthy robot is quiet
#: on the wire): ``{"ok": bool, "base": {...}, "nav": {...},
#: "issues": [str, ...]}``. Issues name the mechanism (e.g.
#: ``"amcl_not_running"``, ``"pose_stale:23s"``, ``"clock_publishers=2"``),
#: not just a boolean, so a client can say *why* rather than just *whether*.
EVENT_SESSION_HEALTH = "session_health"

#: An install, narrated (``{"state", "phase", "percent", "version",
#: "message", "reported_version", "last_update": {...}}``). Pushed only while
#: one runs, and it stops partway through on purpose: the robot_app pushing it
#: is the process docker replaces. ``percent`` is populated only while
#: ``phase`` is ``"pulling"`` — the health gate takes as long as a cold ROS
#: stack takes, and a made-up number for it reads as a stall. What happened
#: after the connection dropped is in ``last_update``, replayed on reconnect.
EVENT_UPDATE_PROGRESS = "update_progress"

#: Continuously-pushed, cached-latest-value telemetry — surfaced through
#: ``read_telemetry()`` / ``wait_for_update()``.
TELEMETRY_EVENTS = frozenset(
    {
        EVENT_POSE,
        EVENT_ODOM,
        EVENT_BATTERY,
        EVENT_JOINT_STATES,
        EVENT_IMU,
        EVENT_MAP,
        EVENT_COSTMAP,
        EVENT_PLAN,
        EVENT_NAV_MODE,
        EVENT_BASE_SESSION,
        EVENT_SESSION_HEALTH,
        EVENT_UPDATE_PROGRESS,
        EVENT_SCAN,
    }
)

#: Discrete async events, not a continuous cache — surfaced via per-topic
#: waiters/queues (e.g. ``wait_for_goal()`` watches ``nav_status``).
ASYNC_EVENTS = frozenset({EVENT_NAV_STATUS, EVENT_DOCK_STATUS})

#: Events replayed by the server on ``auth`` / ``subscribe`` (PROTOCOL.md
#: §3, §5.7) since they're expensive to regenerate.
CACHED_EVENTS = frozenset(
    {
        EVENT_MAP,
        EVENT_COSTMAP,
        EVENT_NAV_MODE,
        EVENT_BASE_SESSION,
        EVENT_SESSION_HEALTH,
        EVENT_UPDATE_PROGRESS,
    }
)

# NOTE: cached-value readers (`get_map()`, `get_plan()`, `get_nav_status()`,
# `system.get_base_session()`) return None on a robot that structurally cannot
# produce the event, which is indistinguishable from "nothing has arrived yet".
# That ambiguity is a known, accepted consequence of removing capability gating
# (PROTOCOL.md §3.1) — documented rather than mechanised.

# --- Errors (PROTOCOL.md §2) -------------------------------------------------

ERROR_NOT_AUTHENTICATED = "not_authenticated"
ERROR_RATE_LIMITED = "rate_limited"
ERROR_INVALID_JSON = "invalid_json"

#: Close code the server uses for a robotId mismatch on the local WS lane
#: (PROTOCOL.md §1) — an optional wrong-robot guard, checked only when the
#: client supplies ``robotId``; not authentication.
CLOSE_CODE_WRONG_ROBOT = 4404

# --- Servo registry — camelCase key -> snake_case URDF joint name -----------
#
# Mirrors ``bonicOS-robot-app/app/config.py``'s ``ACTUATOR_JOINTS`` exactly
# (the server-side single source of truth `servo_command` maps through).
# ``joint_states`` telemetry (PROTOCOL.md §6) reports these snake_case URDF
# names, NOT the camelCase keys a command is sent with — verified against
# the real M1 topic surface (bonicOS-m1-ros sim, 2026-08-04): all 18 keys
# below round-tripped correctly end-to-end through ``bonicOS-robot-app``'s
# WebSocket API. Used both directions: encoding a `servo_command` (camelCase
# in, PROTOCOL.md §5.4) and decoding `joint_states` back into registry keys
# (``ArmController.get_servo_angles()``).
JOINT_NAME_MAP = {
    "rightGripper": "right_gripper_finger1_joint",
    "rightGripperYaw": "right_gripper_yaw_joint",
    "rightWristPitch": "right_wrist_pitch_joint",
    "rightWristYaw": "right_wrist_yaw_joint",
    "rightElbow": "right_elbow_joint",
    "rightShoulderYaw": "right_shoulder_yaw_joint",
    "rightShoulderRoll": "right_shoulder_roll_joint",
    "rightShoulderPitch": "right_shoulder_pitch_joint",
    "leftShoulderPitch": "left_shoulder_pitch_joint",
    "leftShoulderRoll": "left_shoulder_roll_joint",
    "leftShoulderYaw": "left_shoulder_yaw_joint",
    "leftElbow": "left_elbow_joint",
    "leftWristYaw": "left_wrist_yaw_joint",
    "leftWristPitch": "left_wrist_pitch_joint",
    "leftGripperYaw": "left_gripper_yaw_joint",
    "leftGripper": "left_gripper_finger1_joint",
    "neckYaw": "neck_yaw_joint",
    "neckPitch": "neck_pitch_joint",
}

#: The reverse of ``JOINT_NAME_MAP``. ``servo_command``'s and ``head_look``'s
#: ``unsupported`` list names URDF joints (``left_wrist_yaw_joint``), while
#: ``unknown`` echoes the caller's camelCase keys.
REGISTRY_KEY_OF = {joint: key for key, joint in JOINT_NAME_MAP.items()}

# --- Servo registry — camelCase key -> ros2_control controller group --------
#
# Mirrors ``bonicOS-robot-app/app/config.py``'s ``ACTUATOR_JOINTS`` group
# assignment. **Required for correctness, not just bookkeeping**: the
# ``left_arm``/``right_arm`` groups are ``JointTrajectoryController``s, which
# — verified against the real M1 sim (2026-08-04, cross-checked against an
# independent 60-iteration ROS-level stress test in bonicOS-m1-ros) — SILENTLY
# IGNORE a trajectory that omits any of the controller's claimed joints. A
# `servo_command` naming only some of a group's joints (e.g. `move_left_arm`'s
# shoulder+elbow) must have the rest filled in at the group's *current*
# position before sending, or the whole command is a no-op. See
# ``ArmController._send_servo_command``'s ``_fill_group`` step.
JOINT_GROUPS = {
    "left_arm": (
        "leftShoulderYaw",
        "leftShoulderRoll",
        "leftShoulderPitch",
        "leftElbow",
        "leftWristYaw",
        "leftWristPitch",
        "leftGripperYaw",
    ),
    "right_arm": (
        "rightShoulderYaw",
        "rightShoulderRoll",
        "rightShoulderPitch",
        "rightElbow",
        "rightWristYaw",
        "rightWristPitch",
        "rightGripperYaw",
    ),
    "head": ("neckYaw", "neckPitch"),
    "left_gripper": ("leftGripper",),
    "right_gripper": ("rightGripper",),
}

#: Reverse of JOINT_GROUPS — registry key -> its controller group.
JOINT_GROUP_OF = {key: group for group, keys in JOINT_GROUPS.items() for key in keys}

# --- What these tables are NOT: a description of the robot in front of you ---
#
# JOINT_NAME_MAP and JOINT_GROUPS describe the registry's MAXIMUM fitment (the
# 18-actuator M build). They are the vocabulary, not an inventory. A given
# robot fits a subset, and the subset is not even fixed per series:
#
#   - A fits 7 of the 18 (BLE ids 0, 4, 7, 8, 11, 15, 16 — shoulder pitch and
#     elbow per arm, one gripper per side, neck yaw). No wrists, no shoulder
#     yaw/roll, no gripper yaw, NO NECK PITCH. Cross-checked three ways:
#     bonicOS-firmware `bonicbot_actuator_naming.md`, bonicbot-a2-ros
#     `controllers.yaml`, and robot_app's `ROBOT_CONFIG["A"]`.
#   - S fits 14 (adds wrist yaw, shoulder yaw/roll and neck pitch; still no
#     gripper yaw or wrist pitch). No ROS workspace exists for it yet.
#   - M fits all 18, which is why this table looks the way it does.
#
# And fitment is ultimately PER ROBOT, not per series: the ESP loads
# `SERVO_CONFIGURED[]` and a `servo_config` limits blob from NVS at boot
# (bonicOS-firmware `servo_control.cpp`), so two robots of the same series can
# differ. That is precisely why there is no series->joints table in this SDK
# and must not be one — see §3.1 on capability gating. The one honest source
# for "what does THIS robot have" is which joints it reports in
# `joint_states`, which is what `ArmController` fills and waits on.
#
# Server side, a registry joint the robot does not fit comes back in
# `servo_command`'s `unsupported` list (distinct from `unknown`, which is not a
# registry joint at all). Commanding one is not an error — it is dropped, and
# reported, so a client can tell "absent" from "broken".

#: Joint travel limits in DEGREES, as the narrowest range that is valid on
#: every series — i.e. the intersection of A/S/M, not any one robot's range.
#:
#: Source: bonicOS-firmware `bonicbot_actuator_naming.md` + `servo_config.cpp`
#: (compile-time per-series limits), independently confirmed against both
#: URDFs — A2 `gripper.xacro` is [-0.785, 1.047] rad = [-45, 60]deg and M1's
#: is [-1.047, 1.047] = [-60, 60]; M1 `head.xacro` neck yaw is +/-1.5708 =
#: +/-90 and neck pitch +/-0.5236 = +/-30.
#:
#: Used for the SDK's canned poses (``open_grippers``, ``look_left``, ...) so
#: they command something every robot can actually reach. This is NOT
#: validation and must not become clamping of user-supplied angles: the URDF
#: and the ESP both enforce their own limits, and a robot with a wider range
#: should not be held to the narrowest one just because the SDK shipped a
#: table. It exists so a canned pose isn't a guaranteed timeout.
GRIPPER_RANGE_DEG = (-45.0, 60.0)  # A/S -45..60, M -60..60
NECK_YAW_RANGE_DEG = (-90.0, 90.0)  # same on every series

#: Elbow, whose travel is one-sided and **flipped sign on 2026-09-05**.
#:
#: It used to run -50..0 (A), -90..0 (S), -110..0 (M) — zero was the arm
#: straight and every reachable angle was negative. bonicOS-firmware
#: ``678dc38`` ("invert elbow servo range") turned it around, so the same
#: motion is now 0..45 / 0..90 / 0..110 with zero still straight. The A
#: number also moved from 50 to 45 in the same pass, standardizing on the
#: value ``bonicbot-a2-ros`` ``6098b2d`` had already re-signed its URDF to
#: (``body.xacro``/``ros2_control.xacro``, ``lower="0" upper="0.785"``) —
#: rather than widen the URDF to match the old firmware number, the firmware
#: number was brought down to match it, so there is exactly one A-series
#: elbow limit instead of two disagreeing ones.
#:
#: **Anything holding a stored negative elbow angle now means "straight".**
#: A saved sequence, a lesson worksheet or an old snippet written against the
#: previous range clamps at the 0 end and the arm simply does not bend — it
#: does not error, which is what makes this worth writing down. The tuple
#: below is the A range, i.e. the intersection valid on every series; S and M
#: bend further in the same direction.
#:
#: **The ROS lane caught up on A, not on M.** ros2_control clamps a trajectory
#: to the URDF, so whichever range the URDF declares is the one that reaches
#: the arm — and the two workspaces no longer agree:
#:
#: - `bonicbot-a2-ros` ``6098b2d`` re-signed ``body.xacro`` to
#:   ``lower="0" upper="0.785"`` (0..45°) and negated the axis to keep RViz
#:   and Gazebo pointing the same way as the hardware. **A2 works**: a
#:   positive elbow up to 45° bends the arm through the ROS path, matching
#:   this range exactly.
#: - `bonicOS-m1-ros` still declares ``lower="-1.9199" upper="0.0"``. On an M1
#:   a positive elbow is clamped to 0 by ROS while a negative one is clamped
#:   to 0 by the ESP, so the elbow does not move through the ROS path **at
#:   all** and ``wait=True`` times out. Not fixable in the SDK — the SDK does
#:   not clamp, and clamping is not what is wrong.
ELBOW_RANGE_DEG = (0.0, 45.0)  # A 0..45, S 0..90, M 0..110
