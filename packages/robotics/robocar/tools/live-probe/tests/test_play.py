"""Offline tests for play.py — everything except the devices and the session."""

from __future__ import annotations

import live_probe as lp
import play
import pytest
from google.genai import types

# --------------------------------------------------------------------------
# Games, persona and tools
# --------------------------------------------------------------------------


@pytest.mark.parametrize("path", sorted(play.GAMES_DIR.glob("*.toml")))
def test_every_shipped_game_loads(path):
    game = play.load_game(path.read_text(encoding="utf-8"), path.stem)
    assert game.title and game.prompt and game.heartbeat and game.opening


def test_load_game_names_every_missing_field():
    with pytest.raises(ValueError, match="heartbeat, opening"):
        play.load_game('title = "x"\nprompt = "y"\nheartbeat = ""\n', "g")


def test_sound_clip_names_come_from_the_shipped_table():
    names = play.sound_clip_names(play.SOUND_CLIPS_C.read_text(encoding="utf-8"))
    assert names[0] == "horn"
    assert len(names) == len(set(names)) >= 2


def test_sound_clip_names_fails_loudly_without_the_table():
    with pytest.raises(SystemExit, match="SOUND_CLIP_NAMES"):
        play.sound_clip_names("int x;")


def test_play_instruction_layers_persona_register_and_game():
    persona = lp.Persona(name="Teuvo", voice="V", instruction="PERSONA")
    game = play.Game("g", "Peli", "PELIOHJE", "hb", "op")
    text = play.play_instruction(persona, game)
    assert (
        text.index("PERSONA") < text.index(play.PLAY_REGISTER) < text.index("PELIOHJE")
    )
    assert "Peli" in text


def test_play_config_builds_offline_with_play_features_on():
    cfg = play.play_config("ohje", "Schedar", ["horn", "bell"])
    assert isinstance(cfg, types.LiveConnectConfig)
    assert cfg.proactivity.proactive_audio is True
    assert cfg.context_window_compression.sliding_window is not None
    decls = {d.name: d for d in cfg.tools[0].function_declarations}
    assert set(decls) == {"look", "sound", "wait"}
    assert decls["sound"].parameters.properties["name"].enum == ["horn", "bell"]
    assert all(d.behavior == types.Behavior.NON_BLOCKING for d in decls.values())


def test_stub_tool_result_accepts_known_and_rejects_unknown():
    clips = ["horn"]
    assert play.stub_tool_result("sound", {"name": "horn"}, clips)["ok"] is True
    assert play.stub_tool_result("sound", {"name": "kazoo"}, clips)["ok"] is False
    assert play.stub_tool_result("look", {"pan_deg": 10, "tilt_deg": 0}, clips)["ok"]
    assert play.stub_tool_result("wait", {}, clips)["ok"] is True
    assert play.stub_tool_result("drive", {}, clips)["ok"] is False


# --------------------------------------------------------------------------
# Half-duplex gate and heartbeat timing
# --------------------------------------------------------------------------


def test_gate_mutes_through_queued_playback_and_hangover():
    g = play.HalfDuplexGate(hangover_s=0.4)
    g.note_playback(now=10.0, queued_s=1.0)
    g.note_playback(now=10.2, queued_s=0.5)  # queued behind the first chunk
    assert g.playing_until == pytest.approx(11.5)
    assert not g.mic_open(11.6)
    assert g.mic_open(11.9)


def test_gate_flush_reopens_after_hangover_from_the_cut():
    g = play.HalfDuplexGate(hangover_s=0.4)
    g.note_playback(now=0.0, queued_s=5.0)
    g.flush(now=1.0)
    assert not g.is_playing(1.0)
    assert g.mic_open(1.4)


def test_disabled_gate_never_mutes():
    g = play.HalfDuplexGate(hangover_s=0.4, enabled=False)
    g.note_playback(now=0.0, queued_s=5.0)
    assert g.mic_open(1.0)


@pytest.mark.parametrize(
    ("now", "last_hb", "last_sound", "playing", "due"),
    [
        (20.0, 0.0, 0.0, False, True),  # quiet and overdue
        (20.0, 0.0, 0.0, True, False),  # Teuvo is talking
        (20.0, 0.0, 19.0, False, False),  # the child spoke a moment ago
        (20.0, 15.0, 0.0, False, False),  # heartbeat sent recently
    ],
)
def test_heartbeat_due(now, last_hb, last_sound, playing, due):
    assert play.heartbeat_due(now, last_hb, last_sound, playing, 8.0, 2.5) is due


# --------------------------------------------------------------------------
# MJPEG splitting
# --------------------------------------------------------------------------


def _jpeg(body: bytes) -> bytes:
    return play.JPEG_SOI + body + play.JPEG_EOI


def test_split_jpegs_returns_whole_frames_and_keeps_the_partial_one():
    a, b = _jpeg(b"aa"), _jpeg(b"bbb")
    frames, rest = play.split_jpegs(b"junk" + a + b + play.JPEG_SOI + b"cc")
    assert frames == [a, b]
    assert rest == play.JPEG_SOI + b"cc"


def test_split_jpegs_reassembles_a_frame_split_across_reads():
    a = _jpeg(b"payload")
    frames, rest = play.split_jpegs(a[:5])
    assert frames == []
    frames, rest = play.split_jpegs(rest + a[5:])
    assert frames == [a] and rest == b""


# --------------------------------------------------------------------------
# The session log
# --------------------------------------------------------------------------


def _events():
    log = play.SessionLog(t0=100.0)
    log.add(101.0, "heartbeat", 1)
    log.add(102.0, "teuvo", "Hei! ")
    log.add(102.2, "teuvo", "Mitä rakennetaan?")
    log.add(104.0, "child", "Torni")
    log.add(
        105.0, "tool_call", {"name": "sound", "args": {"name": "bell"}, "result": {}}
    )
    log.add(105.5, "turn_complete")
    log.add(170.0, "end", "ctrl-c")
    return log.events


def test_transcript_merges_fragments_and_shows_tools():
    md = play.transcript_markdown(play.Game("g", "Peli", "p", "h", "o"), "m", _events())
    assert "**Teuvo:** Hei! Mitä rakennetaan?" in md
    assert "**Lapsi:** Torni" in md
    assert '`sound({"name": "bell"})`' in md
    assert "`01:10.0` _end: ctrl-c_" in md


def test_session_summary_counts():
    s = play.session_summary(_events())
    assert s["duration_s"] == 70.0
    assert s["end_reason"] == "ctrl-c"
    assert (s["heartbeats"], s["teuvo_turns"], s["tool_calls"]) == (1, 1, 1)
