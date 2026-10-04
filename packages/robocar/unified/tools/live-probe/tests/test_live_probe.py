"""Offline tests for live_probe.py — everything except the network session."""

from __future__ import annotations

import asyncio

import pytest
from google.genai import types

import live_probe as lp

# --------------------------------------------------------------------------
# Model selection
# --------------------------------------------------------------------------

LISTING = {
    "models": [
        {
            "name": "models/gemini-x-flash",
            "supportedGenerationMethods": ["generateContent"],
        },
        {
            "name": "models/gemini-x-live",
            "supportedGenerationMethods": ["bidiGenerateContent"],
        },
        {
            "name": "models/gemini-x-live-translate",
            "supportedGenerationMethods": ["bidiGenerateContent"],
        },
        {
            "name": "models/gemini-x-transcribe-live",
            "supportedGenerationMethods": ["bidiGenerateContent"],
        },
        {"name": "models/embedder"},  # no methods field at all
    ]
}


def test_live_models_keeps_only_bidi_models_and_strips_prefix():
    assert lp.live_models(LISTING) == [
        "gemini-x-live",
        "gemini-x-live-translate",
        "gemini-x-transcribe-live",
    ]


def test_choose_model_skips_single_purpose_live_models():
    assert lp.choose_model(lp.live_models(LISTING)) == "gemini-x-live"


def test_choose_model_refuses_to_guess_between_two_conversational_models():
    with pytest.raises(ValueError, match="pass --model"):
        lp.choose_model(["a-live", "b-live"])


def test_choose_model_fails_when_only_single_purpose_models_exist():
    with pytest.raises(ValueError, match="no conversational"):
        lp.choose_model(["x-live-translate"])


def test_choose_model_accepts_a_listed_request_with_or_without_prefix():
    cands = ["a-live", "b-live"]
    assert lp.choose_model(cands, "b-live") == "b-live"
    assert lp.choose_model(cands, "models/a-live") == "a-live"


def test_choose_model_rejects_a_request_the_key_cannot_reach():
    with pytest.raises(ValueError, match="not a Live model"):
        lp.choose_model(["a-live"], "gemini-made-up-live")


# --------------------------------------------------------------------------
# Audio conversion
# --------------------------------------------------------------------------


def test_read_wav_round_trips_mono_pcm():
    samples = [0, 1000, -1000, 32767, -32768]
    data = lp.wav_bytes(lp.pcm_bytes(samples), 24000)
    assert lp.read_wav(data) == (samples, 24000, 1)


def test_read_wav_rejects_non_16_bit():
    import io
    import wave

    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(1)
        w.setframerate(8000)
        w.writeframes(b"\x80\x80")
    with pytest.raises(ValueError, match="16-bit"):
        lp.read_wav(buf.getvalue())


def test_to_mono_averages_channels():
    assert lp.to_mono([100, 300, -50, 50], 2) == [200, 0]
    assert lp.to_mono([1, 2, 3], 1) == [1, 2, 3]


def test_resample_is_identity_at_the_same_rate():
    s = [5, -7, 9, 11]
    assert lp.resample_linear(s, 16000, 16000) == s


def test_resample_24k_to_16k_keeps_duration_and_dc():
    # One second of DC at 24 kHz must become one second of the same DC at 16 kHz.
    out = lp.resample_linear([1234] * 24000, 24000, 16000)
    assert len(out) == 16000
    assert set(out) == {1234}


def test_resample_interpolates_between_samples():
    # 2 -> 3 points over a ramp: the middle output lands two-thirds along.
    out = lp.resample_linear([0, 300], 2, 3)
    assert out == [0, 200, 300]


def test_resample_upsamples_full_scale_without_overflow():
    # Interpolation between in-range samples stays in range; this pins that a
    # full-scale input still reaches pcm_bytes() as valid int16.
    out = lp.resample_linear([32767, 32767, 32767], 3, 6)
    assert len(out) == 6
    assert set(out) == {32767}
    lp.pcm_bytes(out)  # raises OverflowError on any out-of-range sample


def test_read_wav_rejects_a_truncated_sample():
    data = lp.wav_bytes(lp.pcm_bytes([1, 2, 3]), 16000)
    with pytest.raises(ValueError, match="truncated"):
        lp.read_wav(data[:-1])


def test_chunk_pcm_never_splits_a_sample():
    pcm = bytes(range(256)) * 10  # 2560 bytes = 1280 samples
    chunks = lp.chunk_pcm(pcm, 16000, 10)  # 160 samples = 320 bytes
    assert all(len(c) % 2 == 0 for c in chunks)
    assert len(chunks[0]) == 320
    assert b"".join(chunks) == pcm


def test_parse_pcm_rate():
    assert lp.parse_pcm_rate("audio/pcm;rate=24000") == 24000
    assert lp.parse_pcm_rate("audio/pcm; rate = 16000") == 16000
    assert lp.parse_pcm_rate("audio/pcm") is None
    assert lp.parse_pcm_rate(None) is None
    # A parameter merely ending in "rate" must not be read as the sample rate.
    assert lp.parse_pcm_rate("audio/pcm;bitrate=128") is None


# --------------------------------------------------------------------------
# Persona extraction (against the shipped voice_persona.c, not a fixture)
# --------------------------------------------------------------------------


def test_persona_is_teuvo_from_the_shipped_table():
    persona = lp.extract_persona(lp.PERSONA_C.read_text(encoding="utf-8"))
    assert persona.name == "Teuvo"
    # Robocar's English persona comes first in the table; an unscoped search
    # would return its voice and directive and still look like a success.
    assert "friendly, natural tone" not in persona.instruction
    assert "1950-luvun" in persona.instruction
    assert "suomeksi" in persona.instruction
    assert persona.voice and persona.voice.isalpha()


# --------------------------------------------------------------------------
# Server messages -> events -> report
# --------------------------------------------------------------------------


def _audio_msg(
    data: bytes, mime: str = "audio/pcm;rate=24000"
) -> types.LiveServerMessage:
    return types.LiveServerMessage(
        server_content=types.LiveServerContent(
            model_turn=types.Content(
                role="model",
                parts=[types.Part(inline_data=types.Blob(data=data, mime_type=mime))],
            )
        )
    )


def test_events_from_message_extracts_audio_transcripts_usage_and_turn_end():
    msg = types.LiveServerMessage(
        server_content=types.LiveServerContent(
            input_transcription=types.Transcription(text="hei"),
            output_transcription=types.Transcription(text="päivää"),
            turn_complete=True,
        ),
        usage_metadata=types.UsageMetadata(total_token_count=42),
    )
    kinds = [e.kind for e in lp.events_from_message(msg, 1.0)]
    assert kinds == [
        "input_transcription",
        "output_transcription",
        "turn_complete",
        "usage",
    ]
    audio = lp.events_from_message(_audio_msg(b"\x01\x00\x02\x00"), 2.0)
    assert [(e.kind, e.data[0], e.data[1]) for e in audio] == [
        ("audio", 4, "audio/pcm;rate=24000")
    ]


def test_events_from_message_reports_setup_and_go_away():
    msg = types.LiveServerMessage(
        setup_complete=types.LiveServerSetupComplete(),
        go_away=types.LiveServerGoAway(time_left="50s"),
    )
    kinds = [e.kind for e in lp.events_from_message(msg, 0.0)]
    assert kinds == ["setup_complete", "go_away"]


def _timeline(events: list[lp.Event]) -> lp.Timeline:
    return lp.Timeline(
        t_connect_start=0.0,
        t_speech_end=10.0,
        t_stream_end=10.8,
        clip_seconds=3.0,
        events=events,
    )


def test_report_measures_from_the_end_of_speech():
    ev = [
        lp.Event(0.25, "setup_complete"),
        lp.Event(11.2, "audio", (48000, "audio/pcm;rate=24000", b"")),
        lp.Event(12.2, "audio", (48000, "audio/pcm;rate=24000", b"")),
        lp.Event(12.5, "turn_complete"),
        lp.Event(12.5, "usage", {"total_token_count": 99}),
    ]
    r = lp.build_report(_timeline(ev), "m", "v")
    assert r["connect_to_setup_complete_ms"] == 250
    assert r["first_audio_after_speech_end_ms"] == 1200
    assert r["first_audio_after_stream_end_ms"] == 400
    assert r["turn_complete_after_speech_end_ms"] == 2500
    assert r["output_rate_hz"] == 24000
    assert r["output_audio_bytes"] == 96000
    assert r["output_audio_seconds"] == 2.0
    # 2 s of audio arriving over 1 s of wall clock.
    assert r["arrival_rtf"] == 2.0
    assert r["usage_last"] == {"total_token_count": 99}


def test_report_refuses_to_guess_a_rate_when_chunks_disagree():
    ev = [
        lp.Event(11.0, "audio", (100, "audio/pcm;rate=24000", b"")),
        lp.Event(11.5, "audio", (100, "audio/pcm;rate=16000", b"")),
    ]
    r = lp.build_report(_timeline(ev), "m", "v")
    assert r["output_rate_hz"] is None
    assert r["output_audio_seconds"] is None
    assert r["output_mime_types"] == ["audio/pcm;rate=16000", "audio/pcm;rate=24000"]


def test_report_with_no_audio_has_no_latency():
    r = lp.build_report(_timeline([lp.Event(12.0, "turn_complete")]), "m", "v")
    assert r["first_audio_after_speech_end_ms"] is None
    assert r["output_audio_bytes"] == 0
    assert r["arrival_rtf"] is None


def test_reply_pcm_concatenates_audio_in_order():
    ev = [
        lp.Event(1.0, "audio", (2, "x", b"\x01\x00")),
        lp.Event(1.1, "output_transcription", "x"),
        lp.Event(1.2, "audio", (2, "x", b"\x02\x00")),
    ]
    assert lp.reply_pcm(ev) == b"\x01\x00\x02\x00"


# --------------------------------------------------------------------------
# Session plumbing that does not need the network
# --------------------------------------------------------------------------


def test_stream_paced_stamps_the_last_send_not_its_trailing_sleep():
    now = [0.0]
    sent: list[tuple[float, bytes]] = []

    async def send(chunk: bytes) -> None:
        sent.append((now[0], chunk))

    async def sleep(s: float) -> None:
        now[0] += s

    t = asyncio.run(
        lp.stream_paced(send, [b"a", b"b", b"c"], 0.1, lambda: now[0], sleep)
    )
    assert [c for _, c in sent] == [b"a", b"b", b"c"]
    # Chunks leave at 0.0, 0.1, 0.2; the end of speech is the last send (0.2),
    # not 0.3 after its pacing sleep — that would hide 100 ms of latency.
    assert t == pytest.approx(0.2)
    assert now[0] == pytest.approx(0.3)


def test_stream_paced_with_nothing_to_send_returns_none():
    async def never(_: bytes) -> None:
        raise AssertionError("nothing should be sent")

    assert asyncio.run(lp.stream_paced(never, [], 0.1)) is None


def test_live_config_builds_offline_with_voice_and_persona():
    persona = lp.Persona(name="Teuvo", voice="Charon", instruction="Vastaa suomeksi.")
    cfg = lp.live_config(persona, "Puck")
    assert cfg.response_modalities == [types.Modality.AUDIO]
    assert cfg.speech_config.voice_config.prebuilt_voice_config.voice_name == "Puck"
    assert "suomeksi" in str(cfg.system_instruction)
    assert cfg.input_audio_transcription is not None
    assert cfg.output_audio_transcription is not None


def test_main_fails_fast_without_a_key(monkeypatch, capsys):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    assert lp.main(["--list-models"]) == 2
    assert "GEMINI_API_KEY is not set" in capsys.readouterr().err
