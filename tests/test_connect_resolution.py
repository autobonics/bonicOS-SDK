"""How ``BonicBot()`` decides what to connect to (robot.py module docstring).

The contract these lock down: **``BonicBot()``, written exactly like that,
is a working program in every environment we ship.** A host-registered
transport wins (browser simulator), then explicit arguments (laptop), then
``$BONICOS_HOST``/``$BONICOS_ROBOT_ID`` (the on-robot runner), then mDNS.
If that ordering regresses, the same user file stops running unchanged
across the simulator, the runner, and a developer's machine — which is the
one property the whole Code Studio plan rests on.
"""

from __future__ import annotations

import pytest

import bonicos
from bonicos.exceptions import ConnectionError as BonicConnectionError
from bonicos.robot import BonicBot
from bonicos.transports.mock import MockTransport


@pytest.fixture(autouse=True)
def _clear_injection():
    """Never leak a registration between tests — it's module-global."""
    yield
    bonicos.use_transport(None)


@pytest.fixture(autouse=True)
def _clear_env(monkeypatch):
    for var in ("BONICOS_HOST", "BONICOS_ROBOT_ID", "BONICOS_TOKEN"):
        monkeypatch.delenv(var, raising=False)


@pytest.fixture
def ws_args(monkeypatch):
    """Capture what ``BonicBot`` would have built a real transport with.

    Returns a dict populated on construction, so a test can assert on the
    resolved host/robot_id/token without a socket in the loop.
    """
    captured: dict = {}

    class _FakeWs:
        def __init__(self, host, *, robot_id, token=None):
            captured.update(host=host, robot_id=robot_id, token=token)

        def connect(self, timeout):
            return {
                "robot_id": captured["robot_id"],
                "series": "M",
                "features": {},
            }

    monkeypatch.setattr("bonicos.transports.websocket.WebSocketTransport", _FakeWs)
    return captured


def test_injected_transport_is_adopted_and_connected():
    """The browser path: no host, no robot_id, no socket — still works."""
    transport = MockTransport()
    transport.set_auth_result(robot_id="SIM_001", series="M", features={"nav": True})
    bonicos.use_transport(transport)

    robot = BonicBot()

    assert robot._transport is transport
    assert robot.is_connected()
    # connect() was actually called on it, not just stored.
    assert robot.robot_id == "SIM_001"
    assert robot.series == "M"
    assert robot.features == {"nav": True}


def test_injected_transport_wins_over_arguments_and_env(monkeypatch):
    """Highest precedence: a host that has already decided what this is."""
    monkeypatch.setenv("BONICOS_HOST", "10.0.0.9")
    monkeypatch.setenv("BONICOS_ROBOT_ID", "FROM_ENV")
    transport = MockTransport()
    transport.set_auth_result(robot_id="SIM_001")
    bonicos.use_transport(transport)

    robot = BonicBot("192.168.1.50", robot_id="M1_001")

    assert robot._transport is transport
    assert robot.robot_id == "SIM_001"


def test_env_supplies_host_and_robot_id(monkeypatch, ws_args):
    """The on-robot runner path: user code says BonicBot(), env does the rest."""
    monkeypatch.setenv("BONICOS_HOST", "127.0.0.1")
    monkeypatch.setenv("BONICOS_ROBOT_ID", "M1_001")

    robot = BonicBot()

    assert ws_args == {"host": "127.0.0.1", "robot_id": "M1_001", "token": None}
    assert robot.robot_id == "M1_001"


def test_explicit_arguments_beat_env(monkeypatch, ws_args):
    """A developer naming a robot outranks whatever the shell happens to say."""
    monkeypatch.setenv("BONICOS_HOST", "10.0.0.9")
    monkeypatch.setenv("BONICOS_ROBOT_ID", "FROM_ENV")

    BonicBot("192.168.1.50", robot_id="M1_001")

    assert ws_args["host"] == "192.168.1.50"
    assert ws_args["robot_id"] == "M1_001"


def test_host_and_robot_id_resolve_independently(monkeypatch, ws_args):
    """Mixing sources must work — e.g. runner sets the id, caller names a host."""
    monkeypatch.setenv("BONICOS_ROBOT_ID", "M1_001")

    BonicBot("192.168.1.50")

    assert ws_args["host"] == "192.168.1.50"
    assert ws_args["robot_id"] == "M1_001"


def test_token_falls_back_to_env(monkeypatch, ws_args):
    monkeypatch.setenv("BONICOS_TOKEN", "tok-123")

    BonicBot("192.168.1.50", robot_id="M1_001")

    assert ws_args["token"] == "tok-123"


def test_empty_env_var_is_treated_as_unset(monkeypatch):
    """An exported-but-blank BONICOS_HOST must not become the host string."""
    monkeypatch.setenv("BONICOS_HOST", "")
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot(robot_id="M1_001")

    assert "mDNS" in str(excinfo.value)


def test_missing_robot_id_names_all_three_ways_to_supply_it(monkeypatch):
    """A bare BonicBot() with nothing configured must say what to do next."""
    monkeypatch.setenv("BONICOS_HOST", "127.0.0.1")

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot()

    message = str(excinfo.value)
    assert "robot_id" in message
    assert "BONICOS_ROBOT_ID" in message
    assert "discovery" in message


def test_no_host_anywhere_suggests_passing_one(monkeypatch):
    """mDNS is the last resort; when it finds nothing, say so usefully."""
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot(robot_id="M1_001")

    message = str(excinfo.value)
    assert "mDNS" in message
    assert "BonicBot(" in message  # shows the caller the shape of the fix


def test_use_transport_none_restores_normal_resolution(monkeypatch):
    """Clearing the registration must not leave the SDK wedged in sim mode."""
    bonicos.use_transport(MockTransport())
    bonicos.use_transport(None)
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError):
        BonicBot(robot_id="M1_001")
