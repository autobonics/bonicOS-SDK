"""How ``BonicBot()`` decides what to connect to (robot.py module docstring).

The contract these lock down: **``BonicBot()``, written exactly like that,
is a working program in every environment we ship.** A host-registered
transport wins (browser simulator), then explicit arguments (laptop), then
``$BONICOS_HOST``/``$BONICOS_ROBOT_ID`` (the on-robot runner), then mDNS.
If that ordering regresses, the same user file stops running unchanged
across the simulator, the runner, and a developer's machine — which is the
one property the whole Code Studio plan rests on.

``robot_id`` itself is optional throughout — ``host`` alone is always
enough to connect. These tests cover *resolution* (which host/robot_id wins
when several sources disagree); test_websocket_transport.py and robot_app's
test_local_ws.py cover what the optional ``robotId`` guard actually does at
the wire level.
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
        def __init__(self, host, *, robot_id=None, token=None, uds=None):
            captured.update(host=host, robot_id=robot_id, token=token, uds=uds)

        def connect(self, timeout):
            return {
                "robot_id": captured["robot_id"] or "M1_001",
                "series": "M",
                "features": {},
            }

    monkeypatch.setattr("bonicos.transports.websocket.WebSocketTransport", _FakeWs)
    return captured


def test_injected_transport_is_adopted_and_connected():
    """The browser path: no host, no robot_id, no socket — still works."""
    transport = MockTransport()
    transport.set_auth_result(robot_id="SIM_001", series="M", cameras=["face"])
    bonicos.use_transport(transport)

    robot = BonicBot()

    assert robot._transport is transport
    assert robot.is_connected()
    # connect() was actually called on it, not just stored.
    assert robot.robot_id == "SIM_001"
    assert robot.series == "M"
    assert robot.cameras == ["face"]


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


def test_bare_host_needs_no_robot_id(monkeypatch, ws_args):
    """robot_id is optional — a plain host is a complete, working call."""
    BonicBot("192.168.1.50")

    assert ws_args == {"host": "192.168.1.50", "robot_id": None, "token": None,
                       "uds": None}


def test_env_supplies_host_and_robot_id(monkeypatch, ws_args):
    """The on-robot runner path: user code says BonicBot(), env does the rest."""
    monkeypatch.setenv("BONICOS_HOST", "127.0.0.1")
    monkeypatch.setenv("BONICOS_ROBOT_ID", "M1_001")

    robot = BonicBot()

    assert ws_args == {"host": "127.0.0.1", "robot_id": "M1_001", "token": None,
                       "uds": None}
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

    BonicBot("192.168.1.50")

    assert ws_args["token"] == "tok-123"


def test_empty_env_var_is_treated_as_unset(monkeypatch):
    """An exported-but-blank BONICOS_HOST must not become the host string."""
    monkeypatch.setenv("BONICOS_HOST", "")
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot()

    assert "mDNS" in str(excinfo.value)


def test_no_host_anywhere_suggests_passing_one(monkeypatch):
    """mDNS is the last resort; when it finds nothing, say so usefully."""
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot()

    message = str(excinfo.value)
    assert "mDNS" in message
    assert "BonicBot(" in message  # shows the caller the shape of the fix


def test_no_host_anywhere_names_robot_id_when_one_was_given(monkeypatch):
    """Discovery narrowed to a specific robot and still found nothing — say which."""
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError) as excinfo:
        BonicBot(robot_id="M1_001")

    assert "robot_id='M1_001'" in str(excinfo.value)


def test_use_transport_none_restores_normal_resolution(monkeypatch):
    """Clearing the registration must not leave the SDK wedged in sim mode."""
    bonicos.use_transport(MockTransport())
    bonicos.use_transport(None)
    monkeypatch.setattr("bonicos.discovery.find_robot", lambda robot_id, timeout: None)

    with pytest.raises(BonicConnectionError):
        BonicBot()


# ── the on-robot runner's unix-socket lane ───────────────────────────

def test_uds_env_selects_the_unix_socket_lane(monkeypatch, ws_args):
    """Inside the run_code sandbox there is no network namespace, so
    `BONICOS_UDS` is how a bare `BonicBot()` reaches robot_app at all."""
    monkeypatch.setenv("BONICOS_UDS", "/run/bonic/robot_app.sock")
    monkeypatch.setenv("BONICOS_ROBOT_ID", "A2_001")
    BonicBot()
    assert ws_args["uds"] == "/run/bonic/robot_app.sock"
    assert ws_args["robot_id"] == "A2_001"


def test_uds_is_checked_before_host(monkeypatch, ws_args):
    """Both set (a dev shell on the robot) — the socket wins, because it is
    the one that resolves from inside the sandbox."""
    monkeypatch.setenv("BONICOS_UDS", "/run/bonic/robot_app.sock")
    monkeypatch.setenv("BONICOS_HOST", "127.0.0.1")
    BonicBot()
    assert ws_args["uds"] == "/run/bonic/robot_app.sock"


def test_explicit_host_argument_overrides_the_ambient_socket(monkeypatch, ws_args):
    """An explicit host= is a deliberate override — user code that names a
    second robot must not be silently redirected to the local socket."""
    monkeypatch.setenv("BONICOS_UDS", "/run/bonic/robot_app.sock")
    BonicBot("192.168.1.50")
    assert ws_args["uds"] is None
    assert ws_args["host"] == "192.168.1.50"


def test_uds_url_carries_the_path_and_query_not_a_host(monkeypatch):
    """The server routes on /ws?robotId=; over a socket the authority is a
    placeholder the server ignores."""
    from bonicos.transports.websocket import WebSocketTransport
    t = WebSocketTransport("127.0.0.1", robot_id="A2_001", uds="/run/bonic/s.sock")
    assert t._build_url() == "ws://localhost/ws?robotId=A2_001"
    t_tcp = WebSocketTransport("192.168.1.50", robot_id="A2_001")
    assert t_tcp._build_url() == "ws://192.168.1.50:8080/ws?robotId=A2_001"
