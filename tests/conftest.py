from __future__ import annotations

import pytest

from bonicos.controllers import (
    ArmController,
    CameraController,
    HeadController,
    MotionController,
    NavigationController,
    PreciseMotionController,
    SensorsController,
    SystemController,
)
from bonicos.transports.mock import MockTransport


class FakeRobot:
    """Wires the real controllers to a :class:`MockTransport`.

    Mirrors the composition ``BonicBot.__init__`` does once a transport is
    already connected, without going through environment detection / a real
    socket — that's exactly the seam ``Transport`` exists to let us skip in
    tests.
    """

    def __init__(self, transport: MockTransport) -> None:
        self._transport = transport
        # Handshake-derived attributes the controllers read (robot.py). Kept
        # in step with the real BonicBot deliberately: a double that omits
        # them lets a controller reference an attribute no real robot has.
        # Identity only — there is no `features`/`model`/`variant`/`joints`,
        # because the SDK models no capability (PROTOCOL.md §3.1).
        self.robot_id = "SIM_001"
        self.series = "SIM"
        self.cameras: list = []
        self.motion = MotionController(self)
        self.nav = NavigationController(self)
        self.arm = ArmController(self)
        self.head = HeadController(self)
        self.sensors = SensorsController(self)
        self.system = SystemController(self)
        self.camera = CameraController(self)
        self.precise = PreciseMotionController(self)


@pytest.fixture
def transport() -> MockTransport:
    return MockTransport()


@pytest.fixture
def robot(transport: MockTransport) -> FakeRobot:
    return FakeRobot(transport)
