from __future__ import annotations

from bonicos import protocol


def test_protocol_version_is_an_int() -> None:
    assert isinstance(protocol.PROTOCOL_VERSION, int)
    assert protocol.PROTOCOL_VERSION >= 1


def test_drive_is_unacked() -> None:
    assert protocol.CMD_DRIVE in protocol.UNACKED_COMMANDS


def test_telemetry_and_async_events_are_disjoint() -> None:
    assert protocol.TELEMETRY_EVENTS.isdisjoint(protocol.ASYNC_EVENTS)


def test_nav_status_and_llm_token_are_async_events() -> None:
    assert protocol.EVENT_NAV_STATUS in protocol.ASYNC_EVENTS
    assert protocol.EVENT_LLM_TOKEN in protocol.ASYNC_EVENTS


def test_cached_events_are_telemetry_events() -> None:
    assert protocol.CACHED_EVENTS <= protocol.TELEMETRY_EVENTS


# --- capability is deliberately NOT modelled (PROTOCOL.md §3.1) -------------


def test_no_capability_tables_exist() -> None:
    """The gating framework must stay deleted.

    COMMAND_FEATURES / EVENT_FEATURES / UNGATED_COMMANDS and their
    completeness test were removed: capability is documented, not negotiated,
    and a server with no handler for a command cannot disagree with its own
    hardware the way a mirrored table can. Re-adding any of these reintroduces
    a client-side model of the robot — the exact thing that drifted before.
    """
    for name in (
        "COMMAND_FEATURES",
        "EVENT_FEATURES",
        "UNGATED_COMMANDS",
        "TYPE_FEATURE_UNAVAILABLE",
    ):
        assert not hasattr(
            protocol, name
        ), f"protocol.{name} is back — see PROTOCOL.md §3.1 before restoring it"
    assert not [n for n in vars(protocol) if n.startswith("FEATURE_")]
