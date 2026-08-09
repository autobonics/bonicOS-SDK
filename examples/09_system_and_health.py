"""System-level calls: health, session status, on-device LLM, Wi-Fi, updates
(API.md §10).

`health()`, `get_session_status()`, and `ask_llm()` are safe to run any time.
`reconfig_wifi()`, `trigger_update()`, and `restart_base_session()` are NOT —
one can drop the robot off your network, one restarts the robot_app process,
and one restarts the ROS stack underneath mapping/navigation (~25s+, drops
any WebRTC video peer) — so this example only prints what they'd look like
rather than calling them.

`ask_llm()` is display-only: its output is text you print/speak yourself,
never executed as a command.
"""

from bonicos import BonicBot

HOST = "192.168.29.54"  # robot/tablet IP — e.g. 172.20.10.2 for the Gazebo sim
ROBOT_ID = "M1_001"


def main() -> None:
    with BonicBot(HOST, robot_id=ROBOT_ID) as robot:
        print("health():", robot.health())

        # A fresh, synchronous read of {base, nav, health} — no need to wait
        # for base_session/session_health telemetry to arrive.
        status = robot.get_session_status()
        print("get_session_status():", status)
        if status["health"]["issues"]:
            print("robot is unhealthy:", status["health"]["issues"])

        # Cached telemetry reads — may be None right after connecting if no
        # base_session/session_health frame has arrived yet.
        print("system.get_base_session():", robot.system.get_base_session())
        print("system.get_session_health():", robot.system.get_session_health())

        answer = robot.ask_llm("In one sentence, what is a differential drive robot?")
        print("ask_llm(...) ->", answer)

        print(
            "Not calling reconfig_wifi()/trigger_update()/restart_base_session() "
            "here — disruptive to a running robot (network drop / process "
            "restart / ROS stack restart). Call them yourself when you "
            "actually mean to, e.g.:\n"
            "  robot.reconfig_wifi('my-ssid', 'my-password')\n"
            "  robot.trigger_update()\n"
            "  robot.restart_base_session()  # recover a wedged robot"
        )


if __name__ == "__main__":
    main()
