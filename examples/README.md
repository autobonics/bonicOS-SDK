# bonicos examples

Runnable scripts against a real `BonicBot`, covering one API area each
(API.md is the authoritative reference this mirrors). Each script is
self-contained — no shared helper module.

## Setup

From the `bonicOS-SDK/` directory:

```bash
python -m venv .venv        # or reuse the existing .venv
source .venv/bin/activate
pip install -e ".[native]"  # native = websockets, needed off-robot/off-Pyodide
```

Then open the script you want and edit the `HOST`/`ROBOT_ID` constants near
the top to point at your robot:

```python
HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"
```

```bash
python 01_connect_and_lifecycle.py
```

## What's here

| Script | Covers |
|---|---|
| [`01_connect_and_lifecycle.py`](01_connect_and_lifecycle.py) | `BonicBot()`, `is_connected()`, `features`, `close()`, context-manager form |
| [`02_basic_motion.py`](02_basic_motion.py) | `move_forward/backward`, `turn_left/right`, `drive()`, `stop()`, `is_moving()` |
| [`03_precise_motion_and_queue.py`](03_precise_motion_and_queue.py) | `drive_distance`, `rotate_angle`, `drive_and_rotate`, `draw_square`, `enqueue`/`run_queue`/`clear_queue` |
| [`04_navigation_and_mapping.py`](04_navigation_and_mapping.py) | `go_to`, `navigate_waypoints`, mapping (`start_mapping`/`save_map`/`list_maps`), `get_plan`, locations, `FeatureUnavailable` gating |
| [`05_arms_grippers_neck.py`](05_arms_grippers_neck.py) | `set_servos`, `move_left_arm`/`move_right_arm`, grippers, neck/look, `reset_servos`, `get_servo_angles` |
| [`06_head_expression_and_display.py`](06_head_expression_and_display.py) | `set_expression`, `look`, LED-matrix display — **all 🔌 stub in v1** |
| [`07_speech.py`](07_speech.py) | `speak()` |
| [`08_sensors_and_telemetry.py`](08_sensors_and_telemetry.py) | `get_position/battery/imu`, `wait_for_update` loop pattern, `subscribe()` |
| [`09_system_and_health.py`](09_system_and_health.py) | `health()`, `ask_llm()` (safe); `reconfig_wifi`/`trigger_update` shown, not run |
| [`10_square_patrol_routine.py`](10_square_patrol_routine.py) | A combined routine tying motion + telemetry + speech together |
| [`11_error_handling.py`](11_error_handling.py) | `ConnectionError`, `FeatureUnavailable`, `RobotDisconnected`, `CommandError`, `RobotError` |

## Safety

Scripts that drive the base or arms (`02`, `03`, `04`, `05`, `10`) pause on
`input()` for you to press Enter before sending any actuating command —
Ctrl+C to back out. `09_system_and_health.py` never calls
`reconfig_wifi()`/`trigger_update()` automatically — those can drop the
robot off the network or restart the robot process, so they're only
printed as a pattern to call yourself.

## Running against the sim instead of real hardware

The Gazebo M1 sim exposes the same topic/telemetry surface as the real
robot, so every example works unchanged against it — just set `HOST` to the
sim machine's IP instead.
