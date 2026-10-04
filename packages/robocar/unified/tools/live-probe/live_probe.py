"""Probe the Gemini Live API from a workstation, before any firmware is written.

Issue #619 asks four questions that only a live session can answer: which Live
model the key can reach, how long after the speaker stops the first audio byte
arrives (and in what format), what the session and quota limits are in
practice, and whether Teuvo's Finnish persona survives the move from the
TTS-directive path to a native-audio model. This script opens one session,
streams one clip at real-time pace, and writes down what came back.

Choices that are deliberate rather than incidental:

  * The model id comes from ListModels (``bidiGenerateContent``), never from
    this file. `.claude/rules/gemini-api.md` §1: ids get suffixes and retire,
    and a retired id 404s exactly like a mistyped one.
  * The clip is streamed in 100 ms chunks at wall-clock pace, followed by a
    short run of silence and ``audio_stream_end``. That is how the firmware
    would feed the microphone, so the server's voice-activity detector sees the
    same shape it would see from the robot. Sending the whole file in one
    message would measure a different system.
  * Latency is measured from the end of the *speech* (the last clip sample
    sent), not from the end of the stream. The trailing silence exists for the
    server's VAD; a listener is waiting from the moment they stopped talking.
  * The persona is extracted from ``main/voice_persona.c``, not retyped, with
    the same C-literal reader the style sweep uses. A retyped persona drifts
    from the shipped one and the "does the persona carry over" answer then
    describes text the robot never sends.

Usage:
    uv run python live_probe.py --list-models
    uv run python live_probe.py <input.wav> <out_dir> [--model ID] [--voice NAME]

The key is read from ``GEMINI_API_KEY`` and never printed. Writes
``<out_dir>/reply.wav`` and ``<out_dir>/report.json``.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib.util
import io
import json
import os
import re
import sys
import time
import urllib.request
import wave
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

INPUT_RATE = 16000  # Live API input is natively 16 kHz; it is also the mic's rate.
CHUNK_MS = 100
DEFAULT_TAIL_SILENCE_MS = 800
DEFAULT_TIMEOUT_S = 45.0
LIST_MODELS_URL = "https://generativelanguage.googleapis.com/v1beta/models"

TOOLS_DIR = Path(__file__).resolve().parent.parent
PERSONA_C = TOOLS_DIR.parent / "main" / "voice_persona.c"
STYLE_SWEEP = TOOLS_DIR / "make-style-sweep.py"

# Live models that do one fixed job rather than converse (the deprecations table
# lists a translate and a transcribe model beside the conversational ones). They
# accept a session, so a "pick the only Live model" rule would happily choose
# one and measure the wrong thing.
SINGLE_PURPOSE_MARKERS = ("translate", "transcribe")


# --------------------------------------------------------------------------
# Model selection
# --------------------------------------------------------------------------


def live_models(listing: dict[str, Any]) -> list[str]:
    """Return bare model ids that support ``bidiGenerateContent``, in API order."""
    out: list[str] = []
    for model in listing.get("models", []):
        methods = model.get("supportedGenerationMethods") or []
        if "bidiGenerateContent" in methods:
            out.append(model["name"].removeprefix("models/"))
    return out


def choose_model(candidates: list[str], requested: str | None = None) -> str:
    """Pick the Live model to probe, or fail with the list to choose from.

    An explicit request must be something the key can actually reach; a guess
    that is not in the listing would fail later as an opaque connect error.
    Without a request, exactly one conversational model must remain after the
    single-purpose ones are dropped — two or more is a choice for a human.
    """
    if requested:
        bare = requested.removeprefix("models/")
        if bare not in candidates:
            raise ValueError(
                f"{bare!r} is not a Live model this key can reach; "
                f"ListModels offers: {', '.join(candidates) or '(none)'}"
            )
        return bare
    conversational = [
        c for c in candidates if not any(m in c for m in SINGLE_PURPOSE_MARKERS)
    ]
    if len(conversational) == 1:
        return conversational[0]
    if not conversational:
        raise ValueError(
            "ListModels returned no conversational Live model "
            f"(bidiGenerateContent: {', '.join(candidates) or '(none)'})"
        )
    raise ValueError(
        "several conversational Live models are available; pass --model with one of: "
        + ", ".join(conversational)
    )


def fetch_model_listing(api_key: str) -> dict[str, Any]:
    """Call ListModels, following pagination. The key goes in a header, never the URL."""
    models: list[dict[str, Any]] = []
    token = ""
    while True:
        url = f"{LIST_MODELS_URL}?pageSize=1000" + (
            f"&pageToken={token}" if token else ""
        )
        req = urllib.request.Request(url, headers={"x-goog-api-key": api_key})
        with urllib.request.urlopen(req, timeout=30) as resp:  # noqa: S310 - fixed https host
            page = json.load(resp)
        models.extend(page.get("models", []))
        token = page.get("nextPageToken", "")
        if not token:
            return {"models": models}


# --------------------------------------------------------------------------
# Audio
# --------------------------------------------------------------------------


def read_wav(data: bytes) -> tuple[list[int], int, int]:
    """Decode a 16-bit PCM WAV into (interleaved samples, rate, channels)."""
    with wave.open(io.BytesIO(data), "rb") as w:
        if w.getsampwidth() != 2:
            raise ValueError(f"need 16-bit PCM, got {8 * w.getsampwidth()}-bit")
        rate, channels = w.getframerate(), w.getnchannels()
        frames = w.readframes(w.getnframes())
    samples = list(memoryview(frames).cast("h"))
    return samples, rate, channels


def to_mono(samples: list[int], channels: int) -> list[int]:
    """Average interleaved channels into one."""
    if channels == 1:
        return list(samples)
    return [
        int(round(sum(samples[i : i + channels]) / channels))
        for i in range(0, len(samples) - channels + 1, channels)
    ]


def resample_linear(samples: list[int], src_rate: int, dst_rate: int) -> list[int]:
    """Linear-interpolation resample with int16 clamping.

    Good enough for speech going into a recogniser; it is not an audiophile
    converter and does not need to be. Identity when the rates match, so a
    16 kHz `mic dump` capture reaches the API byte-for-byte unchanged.
    """
    if src_rate == dst_rate or not samples:
        return list(samples)
    n_out = int(round(len(samples) * dst_rate / src_rate))
    last = len(samples) - 1
    out: list[int] = []
    for j in range(n_out):
        pos = j * src_rate / dst_rate
        i = int(pos)
        if i >= last:
            v = float(samples[last])
        else:
            frac = pos - i
            v = samples[i] + (samples[i + 1] - samples[i]) * frac
        out.append(max(-32768, min(32767, int(round(v)))))
    return out


def pcm_bytes(samples: list[int]) -> bytes:
    import array

    a = array.array("h", samples)
    if sys.byteorder == "big":
        a.byteswap()
    return a.tobytes()


def wav_bytes(pcm: bytes, rate: int) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)
    return buf.getvalue()


def chunk_pcm(pcm: bytes, rate: int, ms: int) -> list[bytes]:
    """Split mono 16-bit PCM into chunks of `ms`, never splitting a sample."""
    step = rate * ms // 1000 * 2
    if step <= 0:
        raise ValueError("chunk length rounds to zero samples")
    return [pcm[i : i + step] for i in range(0, len(pcm), step)]


def parse_pcm_rate(mime: str | None) -> int | None:
    """``audio/pcm;rate=24000`` -> 24000. None when no rate parameter is present."""
    if not mime:
        return None
    m = re.search(r"(?:^|;)\s*rate\s*=\s*(\d+)", mime)
    return int(m.group(1)) if m else None


# --------------------------------------------------------------------------
# Persona
# --------------------------------------------------------------------------


def _load_style_sweep():
    spec = importlib.util.spec_from_file_location("make_style_sweep", STYLE_SWEEP)
    if spec is None or spec.loader is None:
        raise SystemExit(f"cannot load {STYLE_SWEEP}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@dataclass(frozen=True)
class Persona:
    name: str
    voice: str
    instruction: str


def extract_persona(source: str) -> Persona:
    """Build Teuvo's Live system instruction from the shipped persona table.

    The Live native-audio models choose the output language themselves and take
    no language code, so the instruction has to carry "answer in Finnish". The
    delivery directive (`tts_style`) has no field of its own here either — it
    rides in the instruction, which is part of what this probe is testing.
    """
    sweep = _load_style_sweep()
    marker = sweep.PERSONA_MARKER
    if marker not in source:
        raise SystemExit(f"voice_persona.c: no {marker!r}; re-point the marker")
    persona = source[source.index(marker) :]
    name = sweep.c_string_after(persona, ".name =")
    voice = sweep.c_string_after(persona, ".voice =")
    brief = sweep.c_string_after(persona, ".text_brief =")
    style = sweep.c_string_after(persona, ".tts_style =")
    if not (name and voice and brief and style):
        raise SystemExit("voice_persona.c: a persona field extracted empty")
    instruction = (
        f"Olet {name}, puhuva robottiauto. {brief}\n"
        f"Ääntämys ja esitystapa: {style}.\n"
        "Vastaa aina suomeksi, ääneen ja lyhyesti."
    )
    return Persona(name=name, voice=voice, instruction=instruction)


# --------------------------------------------------------------------------
# Session events and the report
# --------------------------------------------------------------------------


@dataclass
class Event:
    t: float  # time.monotonic()
    kind: str
    data: Any = None


def events_from_message(msg: Any, t: float) -> list[Event]:
    """Flatten one LiveServerMessage into timestamped events."""
    out: list[Event] = []
    if getattr(msg, "setup_complete", None) is not None:
        out.append(Event(t, "setup_complete"))
    sc = getattr(msg, "server_content", None)
    if sc is not None:
        if sc.model_turn and sc.model_turn.parts:
            for part in sc.model_turn.parts:
                blob = part.inline_data
                if blob is not None and blob.data:
                    out.append(
                        Event(t, "audio", (len(blob.data), blob.mime_type, blob.data))
                    )
                elif part.text:
                    out.append(Event(t, "text", part.text))
        if sc.input_transcription and sc.input_transcription.text:
            out.append(Event(t, "input_transcription", sc.input_transcription.text))
        if sc.output_transcription and sc.output_transcription.text:
            out.append(Event(t, "output_transcription", sc.output_transcription.text))
        if sc.interrupted:
            out.append(Event(t, "interrupted"))
        if sc.generation_complete:
            out.append(Event(t, "generation_complete"))
        if sc.turn_complete:
            out.append(Event(t, "turn_complete"))
    usage = getattr(msg, "usage_metadata", None)
    if usage is not None:
        out.append(Event(t, "usage", usage.model_dump(exclude_none=True, mode="json")))
    go_away = getattr(msg, "go_away", None)
    if go_away is not None:
        out.append(Event(t, "go_away", str(go_away.time_left)))
    return out


@dataclass
class Timeline:
    t_connect_start: float
    t_speech_end: float
    t_stream_end: float
    clip_seconds: float
    events: list[Event] = field(default_factory=list)


def _ms(a: float | None, b: float | None) -> int | None:
    return None if a is None or b is None else int(round((a - b) * 1000))


def build_report(tl: Timeline, model: str, voice: str) -> dict[str, Any]:
    """Turn a session timeline into the numbers issue #619 asks for.

    Pure: no clock, no network. The output rate is taken from the mime type the
    server actually sent, and left None if the chunks disagree — a guessed rate
    turns a format question into a pitch-shifted WAV that sounds plausible.
    """
    ev = tl.events
    first = {
        k: next((e.t for e in ev if e.kind == k), None)
        for k in ("setup_complete", "audio", "turn_complete", "generation_complete")
    }
    audio = [e for e in ev if e.kind == "audio"]
    mimes = sorted({e.data[1] or "" for e in audio})
    rates = {parse_pcm_rate(m) for m in mimes}
    rate = rates.pop() if len(rates) == 1 else None
    n_bytes = sum(e.data[0] for e in audio)
    audio_s = (n_bytes / 2 / rate) if rate else None
    span_s = (audio[-1].t - audio[0].t) if len(audio) > 1 else None
    usages = [e.data for e in ev if e.kind == "usage"]
    return {
        "model": model,
        "voice": voice,
        "clip_seconds": round(tl.clip_seconds, 3),
        "connect_to_setup_complete_ms": _ms(
            first["setup_complete"], tl.t_connect_start
        ),
        "first_audio_after_speech_end_ms": _ms(first["audio"], tl.t_speech_end),
        "first_audio_after_stream_end_ms": _ms(first["audio"], tl.t_stream_end),
        "turn_complete_after_speech_end_ms": _ms(
            first["turn_complete"], tl.t_speech_end
        ),
        "output_mime_types": mimes,
        "output_rate_hz": rate,
        "output_audio_bytes": n_bytes,
        "output_audio_chunks": len(audio),
        "output_audio_seconds": round(audio_s, 3) if audio_s is not None else None,
        # Below 1.0 the stream arrived slower than it plays: a device would need
        # a preroll gate, exactly like the TTS path (see CLAUDE.md, Voice).
        "arrival_rtf": round(audio_s / span_s, 2) if audio_s and span_s else None,
        "interrupted": any(e.kind == "interrupted" for e in ev),
        "input_transcription": "".join(
            e.data for e in ev if e.kind == "input_transcription"
        ),
        "output_transcription": "".join(
            e.data for e in ev if e.kind == "output_transcription"
        ),
        "text_parts": "".join(e.data for e in ev if e.kind == "text"),
        "usage_last": usages[-1] if usages else None,
        "usage_messages": len(usages),
        "go_away_time_left": next((e.data for e in ev if e.kind == "go_away"), None),
    }


def reply_pcm(events: list[Event]) -> bytes:
    return b"".join(e.data[2] for e in events if e.kind == "audio")


# --------------------------------------------------------------------------
# The session (network; not unit-tested — this is the part the probe measures)
# --------------------------------------------------------------------------


async def run_session(
    api_key: str,
    model: str,
    persona: Persona,
    voice: str,
    pcm16k: bytes,
    tail_silence_ms: int,
    timeout_s: float,
) -> Timeline:
    from google import genai
    from google.genai import types

    client = genai.Client(api_key=api_key)
    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice)
            )
        ),
        system_instruction=persona.instruction,
        input_audio_transcription=types.AudioTranscriptionConfig(),
        output_audio_transcription=types.AudioTranscriptionConfig(),
    )
    mime = f"audio/pcm;rate={INPUT_RATE}"
    events: list[Event] = []
    t0 = time.monotonic()
    async with client.aio.live.connect(model=model, config=config) as session:
        events.append(Event(time.monotonic(), "connected"))

        async def receive() -> None:
            while True:
                async for msg in session.receive():
                    now = time.monotonic()
                    batch = events_from_message(msg, now)
                    events.extend(batch)
                    if any(e.kind == "turn_complete" for e in batch):
                        return

        receiver = asyncio.create_task(receive())
        # Real-time pacing: one chunk per chunk-length of wall clock.
        for chunk in chunk_pcm(pcm16k, INPUT_RATE, CHUNK_MS):
            await session.send_realtime_input(
                audio=types.Blob(data=chunk, mime_type=mime)
            )
            await asyncio.sleep(CHUNK_MS / 1000)
        t_speech_end = time.monotonic()
        silence = bytes(INPUT_RATE * tail_silence_ms // 1000 * 2)
        for chunk in chunk_pcm(silence, INPUT_RATE, CHUNK_MS) if silence else []:
            await session.send_realtime_input(
                audio=types.Blob(data=chunk, mime_type=mime)
            )
            await asyncio.sleep(CHUNK_MS / 1000)
        await session.send_realtime_input(audio_stream_end=True)
        t_stream_end = time.monotonic()
        try:
            await asyncio.wait_for(receiver, timeout=timeout_s)
        except TimeoutError:
            events.append(Event(time.monotonic(), "timeout"))
            receiver.cancel()
    return Timeline(
        t_connect_start=t0,
        t_speech_end=t_speech_end,
        t_stream_end=t_stream_end,
        clip_seconds=len(pcm16k) / 2 / INPUT_RATE,
        events=events,
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "input", nargs="?", type=Path, help="16-bit PCM WAV, any rate/channels"
    )
    p.add_argument("out_dir", nargs="?", type=Path)
    p.add_argument("--model", help="Live model id; must appear in ListModels")
    p.add_argument("--voice", help="prebuilt voice (default: the persona's own)")
    p.add_argument(
        "--list-models", action="store_true", help="print Live models and exit"
    )
    p.add_argument("--tail-silence-ms", type=int, default=DEFAULT_TAIL_SILENCE_MS)
    p.add_argument("--timeout-s", type=float, default=DEFAULT_TIMEOUT_S)
    args = p.parse_args(argv)

    api_key = os.environ.get("GEMINI_API_KEY", "")
    if not api_key:
        print(
            "GEMINI_API_KEY is not set (expected from ~/.api_tokens via mise)",
            file=sys.stderr,
        )
        return 2

    candidates = live_models(fetch_model_listing(api_key))
    if args.list_models:
        print("\n".join(candidates) or "(no bidiGenerateContent models)")
        return 0
    if args.input is None or args.out_dir is None:
        p.error("input and out_dir are required unless --list-models is given")
    try:
        model = choose_model(candidates, args.model)
    except ValueError as err:
        print(f"model selection: {err}", file=sys.stderr)
        return 2

    samples, rate, channels = read_wav(args.input.read_bytes())
    pcm16k = pcm_bytes(resample_linear(to_mono(samples, channels), rate, INPUT_RATE))
    persona = extract_persona(PERSONA_C.read_text(encoding="utf-8"))
    voice = args.voice or persona.voice
    print(
        f"model={model} voice={voice} clip={len(pcm16k) / 2 / INPUT_RATE:.2f}s "
        f"(from {rate} Hz x{channels})",
        file=sys.stderr,
    )

    tl = asyncio.run(
        run_session(
            api_key, model, persona, voice, pcm16k, args.tail_silence_ms, args.timeout_s
        )
    )
    report = build_report(tl, model, voice)
    report["timed_out"] = any(e.kind == "timeout" for e in tl.events)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    if report["output_rate_hz"]:
        (args.out_dir / "reply.wav").write_bytes(
            wav_bytes(reply_pcm(tl.events), report["output_rate_hz"])
        )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["output_audio_bytes"] and not report["timed_out"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
