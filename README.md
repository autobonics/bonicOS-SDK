# bonicos

**The Python SDK for BonicBot robots.** Drive the base, move the arms, run
navigation and mapping, and read live sensor telemetry — from your own laptop
or from a program running on the robot itself, with the same code either way.

```bash
pip install bonicos
```

```python
from bonicos import BonicBot

with BonicBot("192.168.1.50", robot_id="M1_001") as robot:
    robot.move_forward(speed=0.3, duration=2)
    robot.move_left_arm(shoulder=45, elbow=-30)
    print(robot.get_battery(), "V")
# motors stopped and the connection closed, even on an exception
```

Requires Python 3.10+. Pure Python — the same wheel runs on a laptop, a
Raspberry Pi, or a Jetson.

---

## Connecting

`bonicos` talks to the robot over your local network. You need the robot's
address and its robot id:

```python
robot = BonicBot("192.168.1.50", robot_id="M1_001")
```

The id must match the robot you're pointing at — it's a guard against driving
the wrong machine on a network with several robots on it, and the connection is
refused if it doesn't match.

If the environment already knows which robot you mean, a bare `BonicBot()`
works. That's the case when your program runs **on the robot**, where
`BONICOS_HOST` and `BONICOS_ROBOT_ID` are set for you — so a script you
developed on your laptop needs no edits to run on the robot. You can also set
those variables yourself, or install `bonicos[discovery]` to find a robot by
mDNS.

> **Anyone on the same network can connect.** There is no authentication yet —
> the robot id is a wrong-robot guard, not a password. Run robots on a network
> you trust.

No robot handy? `BonicBot.simulated()` connects to a fake one instead —
driving, arms, and telemetry all behave for real, with no network and no
hardware required.

---

## What you can do

| Area | Examples |
|---|---|
| **Motion** | `move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`, raw `drive` |
| **Precise motion** | `drive_distance`, `rotate_angle`, `drive_and_rotate`, `draw_square`, queued routines |
| **Navigation** | `go_to(x, y)`, `navigate_waypoints`, `cancel_goal`, `wait_for_goal`, `get_plan` |
| **Mapping** | `enter_mapping_mode`, `start_mapping`, `save_map`, `list_maps`, `enter_navigation_mode` |
| **Arms & grippers** | `move_left_arm`, `move_right_arm`, `set_servos`, `set_gripper`, `set_neck`, `get_servo_angles` |
| **Sensors** | `get_position`, `get_battery`, `get_imu`, `get_servo_angles`, `wait_for_update` |
| **Camera** | `get_camera_frame()` → BGR numpy arrays (needs `pip install bonicos[camera]`) |
| **System** | `speak`, `health`, `ask_llm`, session status and recovery |

Full reference with every signature: **[API.md](./API.md)**.

Movement calls block until the robot actually gets there — `move_left_arm(...)`
returns when the joint has converged on its target, not when the command was
merely acknowledged.

### Not yet working on the robot

Some commands are accepted and silently do nothing on current robot firmware.
Your code runs; that actuator just doesn't move. They are marked **🔌 stub** in
[API.md](./API.md):

- Named locations — `save_location`, `goto_location`, `list_locations`,
  `delete_location`, `delete_all_locations`
- Nav2 lifecycle — `start_navigation`, `stop_navigation`
- `servo_single`, head expression (`head_mode`, `head_look`), and the LED
  matrix (`display_*`)

Everything else in the table above is live. Vision pipelines (face/pose/object
detection), autonomous exploration, and recorded sequences are not in this
release.

---

## Safety

- Use the context-manager form (`with BonicBot(...) as robot:`) or call
  `robot.close()` in a `finally`. Both stop the robot on the way out.
- The robot stops its own motors if it stops hearing from you (roughly 400 ms),
  so a crashed script or a dropped connection will not leave it driving. That
  backstop covers driving only — a navigation goal keeps running, so cancel it
  explicitly if you're bailing out.

---

## Installing extras

```bash
pip install bonicos              # driving, arms, navigation, telemetry
pip install bonicos[camera]      # + camera frames (aiortc, numpy)
pip install bonicos[discovery]   # + find a robot by mDNS
```

Each extra reports what's missing if you use a feature without it, rather than
failing with an import error.

---

## Documentation

- **[API.md](./API.md)** — every class and method, with blocking behaviour and
  worked examples.
- **[PROTOCOL.md](./PROTOCOL.md)** — the wire protocol, if you're writing your
  own client or working on the robot side.

## License

MIT — see [LICENSE](./LICENSE). © Autobonics Pvt Ltd.
