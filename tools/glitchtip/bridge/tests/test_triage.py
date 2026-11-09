"""Formatting of GlitchTip issues and events, against responses recorded from
GlitchTip 6.2.6 after `just glitchtip::send` of each fixture."""

from __future__ import annotations

import json
from pathlib import Path

from robocar_glitchtip_bridge.triage import format_event, format_issues

FIX = Path(__file__).parent / "fixtures"


def load(name: str):
    return json.loads((FIX / name).read_text())


def event_text(n: int) -> str:
    d = load(f"event-{n}.json")
    return format_event(d["issue"], d["event"])


def test_issue_list_has_one_line_per_issue_with_the_key_facts():
    lines = format_issues(load("issues.json")).splitlines()
    assert len(lines) == 3
    crash = next(line for line in lines if line.startswith("#2 "))
    assert "robocar " in crash
    assert "fatal" in crash
    assert "1x" in crash
    assert "robocar-unified@0.0.0-fixture" in crash
    assert crash.endswith("LoadProhibited: PC 0x0000000a")
    note = next(line for line in lines if line.startswith("#3 "))
    assert "robocar-ai" in note


def test_empty_issue_list_says_so():
    assert format_issues([]) == "no issues"


def test_crash_event_shows_symbolicated_frames_innermost_first():
    text = event_text(2)
    assert "Exception: LoadProhibited: PC 0x0000000a" in text
    frames = [line.strip() for line in text.splitlines() if line.startswith("    0x")]
    assert frames[0].startswith("0x0000000a add tools/glitchtip/fixtures/tiny.c:3")
    assert frames[1].startswith("0x0000002c entry tools/glitchtip/fixtures/tiny.c:8")
    assert "symbolicated=true" in text
    assert "reset_reason=PANIC" in text


def test_reset_event_shows_breadcrumbs_and_extra():
    text = event_text(1)
    assert "Message: reset: BROWNOUT" in text
    assert "[log] I (4100) motor: start L=180 R=180" in text
    assert "uptime_ms=4210" in text
    assert "Exception" not in text


def test_planner_note_shows_trigger_and_kind():
    text = event_text(3)
    assert "[robocar-ai]" in text
    assert "Message: The chair leg keeps tripping the reflex when I turn left." in text
    assert "trigger=reflex" in text
    assert "kind=difficulty" in text
    assert "reflex_trips=3" in text
    assert "[goal] stop" in text


def test_server_side_tags_are_left_out():
    text = event_text(1)
    assert "server_name" not in text
    assert "environment=" not in text


def test_robot_without_a_clock_is_flagged():
    assert "bridge receive time" in event_text(1)


def test_unsymbolicated_frame_shows_the_address_alone():
    d = load("event-2.json")
    exc = next(e for e in d["event"]["entries"] if e["type"] == "exception")
    exc["data"]["values"][0]["stacktrace"]["frames"] = [
        {"instruction_addr": "0x42001234"}
    ]
    text = format_event(d["issue"], d["event"])
    assert "    0x42001234 ?" in text
