"""Offline tests for generate.py: the request it puts on the wire, and the audio
it accepts back. No key, no network: the SDK client talks to an httpx
MockTransport, so a test sees the exact JSON body and URL the SDK would send.

Pinning the wire body (not just our own arguments) is the point. The 3.8 style
field only exists in newer SDKs, and an SDK whose converter silently dropped
`speech_metadata` would turn every styled clip into an unstyled one with no
error anywhere.
"""

from __future__ import annotations

import base64
import io
import json
import re
import struct
import wave

import generate as g
import httpx
import pytest
from google import genai
from google.genai import types

PCM = b"".join(i.to_bytes(2, "little", signed=True) for i in range(-8, 8))


def _wav(pcm: bytes, rate: int = 24000, channels: int = 1, width: int = 2) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(width)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


def _client(seen: list[dict], data: bytes, mime: str) -> genai.Client:
    def handler(req: httpx.Request) -> httpx.Response:
        seen.append({"url": str(req.url), "body": json.loads(req.content)})
        part = {
            "inlineData": {"mimeType": mime, "data": base64.b64encode(data).decode()}
        }
        return httpx.Response(
            200, json={"candidates": [{"content": {"parts": [part]}}]}
        )

    return genai.Client(
        api_key="offline-test",
        http_options=types.HttpOptions(
            client_args={"transport": httpx.MockTransport(handler)}
        ),
    )


def _camel(obj):
    """Spell every key lowerCamelCase. SDK 2.34 sends speechConfig's nested keys
    as proto field names (`voice_config`); proto3 JSON parsers accept both
    spellings, so the test pins the meaning and not the spelling."""
    if isinstance(obj, dict):
        return {
            re.sub(r"_([a-z])", lambda m: m.group(1).upper(), k): _camel(v)
            for k, v in obj.items()
        }
    if isinstance(obj, list):
        return [_camel(v) for v in obj]
    return obj


def _part(body: dict) -> dict:
    (content,) = body["contents"]
    (part,) = content["parts"]
    return part


# --------------------------------------------------------------------------
# The request
# --------------------------------------------------------------------------


def test_model_is_the_3_8_tts_model_the_firmware_calls():
    # gemini-3.1-flash-tts-preview has a listed shutdown of 2026-11-17 (#844).
    # The firmware's TTS_MODEL (main/tts_request.h) is the reference.
    assert g.MODEL == "gemini-3.8-flash-tts"


def test_style_goes_in_speech_metadata_and_the_text_stays_verbatim():
    seen: list[dict] = []
    g.synthesize(
        _client(seen, PCM, "audio/L16;codec=pcm;rate=24000"), "Hei.", "Kore", "Hitaasti"
    )
    (req,) = seen
    assert req["url"].endswith("/models/gemini-3.8-flash-tts:generateContent")
    part = _part(req["body"])
    # 3.8 reads the text as a verbatim transcript: a "<style>: <text>" prefix
    # would be spoken. Same shape as tts_request_build() in the firmware.
    assert part["text"] == "Hei."
    assert part["speechMetadata"] == {"style": "Hitaasti"}


def test_no_style_sends_no_speech_metadata():
    seen: list[dict] = []
    g.synthesize(_client(seen, PCM, "audio/L16;codec=pcm;rate=24000"), "Hei.", "Kore")
    part = _part(seen[0]["body"])
    assert part == {"text": "Hei."}


def test_voice_and_language_code_reach_speech_config():
    seen: list[dict] = []
    g.synthesize(
        _client(seen, PCM, "audio/L16;codec=pcm;rate=24000"),
        "Hei.",
        "Charon",
        "",
        "fi-FI",
    )
    cfg = _camel(seen[0]["body"]["generationConfig"])
    assert cfg["responseModalities"] == ["AUDIO"]
    assert cfg["speechConfig"]["languageCode"] == "fi-FI"
    assert (
        cfg["speechConfig"]["voiceConfig"]["prebuiltVoiceConfig"]["voiceName"]
        == "Charon"
    )


def test_no_language_code_omits_the_field():
    seen: list[dict] = []
    g.synthesize(_client(seen, PCM, "audio/L16;codec=pcm;rate=24000"), "Hei.", "Kore")
    assert "languageCode" not in _camel(
        seen[0]["body"]["generationConfig"]["speechConfig"]
    )


# --------------------------------------------------------------------------
# The response: bare PCM out, whichever form the model returned
# --------------------------------------------------------------------------


def test_bare_l16_at_24k_passes_through():
    out = g.synthesize(
        _client([], PCM, "audio/L16;codec=pcm;rate=24000"), "Hei.", "Kore"
    )
    assert out == PCM


def test_wav_is_unwrapped_to_bare_pcm():
    # The 3.8 docs say a unary request returns WAV. The firmware clips are raw
    # PCM, so a RIFF header must not reach them as 44 bytes of click.
    out = g.synthesize(_client([], _wav(PCM), "audio/wav"), "Hei.", "Kore")
    assert out == PCM


def test_wav_is_detected_by_its_header_even_under_another_mime():
    out = g.synthesize(_client([], _wav(PCM), "audio/L16;rate=24000"), "Hei.", "Kore")
    assert out == PCM


@pytest.mark.parametrize(
    ("data", "mime"),
    [
        (PCM, "audio/L16;codec=pcm;rate=16000"),
        (_wav(PCM, rate=16000), "audio/wav"),
        (_wav(PCM + PCM, channels=2), "audio/wav"),
        (_wav(PCM[:15], width=1), "audio/wav"),
        (PCM, "audio/mulaw;rate=8000"),
    ],
    ids=["l16-16k", "wav-16k", "wav-stereo", "wav-8bit", "mulaw"],
)
def test_audio_the_player_cannot_play_fails_loudly(data, mime):
    # Every consumer plays 24 kHz s16 mono. Writing another format would
    # produce a clip that plays at the wrong pitch or as noise.
    with pytest.raises(g.AudioFormatError):
        g.synthesize(_client([], data, mime), "Hei.", "Kore")


def test_l16_without_a_rate_is_taken_as_24k():
    # The preview sent `audio/L16;codec=pcm;rate=24000`; a bare `audio/l16`
    # (the firmware's streaming form) carries no rate. 24 kHz is documented
    # as the only output rate.
    out = g.synthesize(_client([], PCM, "audio/l16"), "Hei.", "Kore")
    assert out == PCM


# --------------------------------------------------------------------------
# --wav output
# --------------------------------------------------------------------------


def test_wav_bytes_round_trips_through_the_wave_module():
    with wave.open(io.BytesIO(g.wav_bytes(PCM)), "rb") as w:
        assert (w.getframerate(), w.getnchannels(), w.getsampwidth()) == (24000, 1, 2)
        assert w.readframes(w.getnframes()) == PCM


# --------------------------------------------------------------------------
# Empty and unreadable audio (review of PR #994)
# --------------------------------------------------------------------------


def _float_wav() -> bytes:
    """A WAV with format tag 3 (IEEE float), which the wave module refuses."""
    fmt = struct.pack("<HHIIHH", 3, 1, 24000, 96000, 4, 32)
    data = b"\0" * 8
    body = b"WAVE" + b"fmt " + struct.pack("<I", len(fmt)) + fmt
    body += b"data" + struct.pack("<I", len(data)) + data
    return b"RIFF" + struct.pack("<I", len(body)) + body


def test_an_unreadable_wav_is_an_audio_format_error():
    with pytest.raises(g.AudioFormatError):
        g.synthesize(_client([], _float_wav(), "audio/wav"), "Hei.", "Kore")


def test_an_empty_wav_is_retried_not_written(monkeypatch):
    # A header-only WAV unwraps to no PCM. Before the 3.8 move an empty
    # response was retried; writing a 0-byte clip instead fails silently.
    monkeypatch.setattr(g.time, "sleep", lambda _s: None)
    replies = [_wav(b""), _wav(PCM)]
    seen: list[dict] = []

    def handler(req: httpx.Request) -> httpx.Response:
        seen.append({})
        data = replies.pop(0)
        part = {
            "inlineData": {
                "mimeType": "audio/wav",
                "data": base64.b64encode(data).decode(),
            }
        }
        return httpx.Response(
            200, json={"candidates": [{"content": {"parts": [part]}}]}
        )

    client = genai.Client(
        api_key="offline-test",
        http_options=types.HttpOptions(
            client_args={"transport": httpx.MockTransport(handler)}
        ),
    )
    assert g.synthesize(client, "Hei.", "Kore") == PCM
    assert len(seen) == 2
