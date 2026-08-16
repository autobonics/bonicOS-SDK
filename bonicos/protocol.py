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
#: BaseSessionManager. ``restart_base_session`` is feature-gated on
#: ``session_control`` and refused while the robot is moving or navigating
#: (cancel/stop first); it is slow (a cold Gazebo start is ~25s, on top of
#: nav teardown/reseed) so the SDK uses a much longer default timeout than
#: other commands. ``get_session_status`` is a synchronous, ungated
#: point-in-time read of the base+nav+health state — the same information
#: the ``base_session``/``session_health`` telemetry events push on change.
CMD_RESTART_BASE_SESSION = "restart_base_session"
CMD_GET_SESSION_STATUS = "get_session_status"

#: §5.3 Named locations — all stub in v1.
CMD_SAVE_LOCATION = "save_location"
CMD_GOTO_LOCATION = "goto_location"
CMD_DELETE_LOCATION = "delete_location"
CMD_DELETE_ALL_LOCATIONS = "delete_all_locations"
CMD_LIST_LOCATIONS = "list_locations"

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

#: §5.7 System & session.
CMD_HEALTH = "health"
CMD_RECONFIG_WIFI = "reconfig_wifi"
CMD_TRIGGER_UPDATE = "trigger_update"
CMD_SUBSCRIBE = "subscribe"
CMD_LLM_QUERY = "llm_query"

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
EVENT_LLM_TOKEN = "llm_token"

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
    }
)

#: Discrete async events, not a continuous cache — surfaced via per-topic
#: waiters/queues (e.g. ``wait_for_goal()`` watches ``nav_status``).
ASYNC_EVENTS = frozenset({EVENT_NAV_STATUS, EVENT_LLM_TOKEN})

#: Events replayed by the server on ``auth`` / ``subscribe`` (PROTOCOL.md
#: §3, §5.7) since they're expensive to regenerate.
CACHED_EVENTS = frozenset(
    {EVENT_MAP, EVENT_COSTMAP, EVENT_NAV_MODE, EVENT_BASE_SESSION, EVENT_SESSION_HEALTH}
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
