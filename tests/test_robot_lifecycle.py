from __future__ import annotations

import pytest

from bonicos.controllers import (
    ArmController,
    HeadController,
    MotionController,
    NavigationController,
    PreciseMotionController,
    SensorsController,
    SystemController,
)
from bonicos.robot import BonicBot
from bonicos.transports.mock import MockTransport


def _make_bonicbot(transport: MockTransport) -> BonicBot:
    """Build a real ``BonicBot`` around an already-constructed transport,
    bypassing ``__init__``'s environment detection (which would otherwise
    try to build a real WebSocket/WebRTC transport) — mirrors exactly what
    ``__init__`` does once a transport exists.
    """
    bot = BonicBot.__new__(BonicBot)
    auth_result = transport.connect(10.0)
    bot._transport = transport
    bot.robot_id = auth_result.get("robot_id", "")
    bot.series = auth_result.get("series", "")
    bot.features = dict(auth_result.get("features", {}) or {})
    bot._connected = True
    bot.motion = MotionController(bot)
    bot.nav = NavigationController(bot)
    bot.arm = ArmController(bot)
    bot.head = HeadController(bot)
    bot.sensors = SensorsController(bot)
    bot.system = SystemController(bot)
    bot._precise = PreciseMotionController(bot)
    return bot


def test_is_connected_toggles_on_close() -> None:
    bot = _make_bonicbot(MockTransport())
    assert bot.is_connected() is True
    bot.close()
    assert bot.is_connected() is False


def test_close_is_idempotent() -> None:
    bot = _make_bonicbot(MockTransport())
    bot.close()
    bot.close()  # must not raise


def test_context_manager_stops_and_closes_even_on_exception() -> None:
    transport = MockTransport()
    bot = _make_bonicbot(transport)
    with pytest.raises(ValueError):
        with bot:
            bot.move_forward(0.3, duration=None)
            raise ValueError("boom")

    assert bot.is_connected() is False
    drive_frames = [m for m in transport.sent if m["type"] == "drive"]
    assert drive_frames[-1] == {"type": "drive", "linear_x": 0.0, "angular_z": 0.0}


def test_flat_methods_delegate_to_grouped_controllers() -> None:
    transport = MockTransport()
    bot = _make_bonicbot(transport)
    bot.drive(0.2, 0.1)
    assert transport.sent[-1] == {"type": "drive", "linear_x": 0.2, "angular_z": 0.1}
    bot.stop()
