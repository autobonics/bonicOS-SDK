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

    def __init__(self, transport: MockTransport, features: dict | None = None) -> None:
        self._transport = transport
        self.features = features or {}
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
