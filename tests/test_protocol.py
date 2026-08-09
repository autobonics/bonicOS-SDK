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
