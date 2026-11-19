"""Interactive play with Teuvo over the Gemini Live API, from a workstation.

The latency probe (`live_probe.py`) answers "how fast, in what format". This
answers "is it fun": Teuvo watches through a camera, listens and talks back
while a child plays (Lego, Moomin figures), and every session leaves a
transcript to tune the game prompt against. No board is involved; tool calls
(look, sound) are logged and answered as if the robot had done them.

Choices that are deliberate rather than incidental:

  * Frames go in at 1 per second, the documented Live API video rate, and a
    frame alone never starts a reply. A *heartbeat* (the game's short "look
    again" text) is sent only when the session has been quiet for a while, so
    it never barges in on Teuvo or on the child (`heartbeat_due`).
  * Proactive audio is on, so Teuvo may stay silent after a heartbeat. Without
    it, every heartbeat would get a comment and play would turn into narration.
  * The microphone is muted while Teuvo's reply plays, plus a short hangover
    (`HalfDuplexGate`). Laptop speakers into a laptop mic otherwise make Teuvo
    interrupt himself. `--barge-in` turns the gate off for headphones; the
    robot faces the same choice (issue #619, "Echo").
  * Context-window compression is on: without it an audio+video session ends
    after 2 minutes.
  * The persona comes from `main/voice_persona.c` and the sound names from
    `main/sound_clips_core.c`, extracted, not retyped, so what is tuned here is
    what the robot would send.

Usage:
    uv run --extra play python play.py --list-devices
    uv run --extra play python play.py <out_root> --game games/building.fi.toml

Ends on Ctrl-C, `--max-minutes`, or the server's GoAway, and writes
``<out_root>/<UTC stamp>-<game>/``: transcript.md, events.jsonl, session.json,
teuvo.wav, mic.wav and frames/.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import datetime as dt
import json
import os
import re
import sys
import time
import tomllib
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import live_probe as lp

INPUT_RATE = lp.INPUT_RATE  # 16 kHz mono s16le to the server
OUTPUT_RATE = 24000  # documented, and measured on 2026-11-07 (README § Measured)
MIC_BLOCK_MS = 50
FRAME_INTERVAL_S = 1.0  # documented Live API video input limit
DEFAULT_HEARTBEAT_S = 8.0
DEFAULT_QUIET_S = 2.5  # since the child or Teuvo last made a sound
DEFAULT_HANGOVER_MS = 400  # mic stays muted this long after Teuvo's audio ends
DEFAULT_MAX_MINUTES = 10.0  # one connection lasts about 10 min (GoAway)

SOUND_CLIPS_C = lp.TOOLS_DIR.parent / "main" / "sound_clips_core.c"
GAMES_DIR = Path(__file__).resolve().parent / "games"

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


# --------------------------------------------------------------------------
# Games and the system instruction
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Game:
    slug: str
    title: str
    prompt: str
    heartbeat: str
    opening: str


def load_game(text: str, slug: str) -> Game:
    """Parse a game TOML. Every field is required: an empty heartbeat would
    send the model an empty turn, and an empty prompt plays the plain persona."""
    data = tomllib.loads(text)
    missing = [
        k for k in ("title", "prompt", "heartbeat", "opening") if not data.get(k)
    ]
    if missing:
        raise ValueError(f"game {slug!r}: missing or empty {', '.join(missing)}")
    return Game(
        slug=slug,
        title=data["title"].strip(),
        prompt=data["prompt"].strip(),
        heartbeat=data["heartbeat"].strip(),
        opening=data["opening"].strip(),
    )


def sound_clip_names(source: str) -> list[str]:
    """Bare clip names from ``SOUND_CLIP_NAMES`` in sound_clips_core.c, in order."""
    m = re.search(r"SOUND_CLIP_NAMES\[[^\]]*\]\s*=\s*\{(.*?)\};", source, re.S)
    if not m:
        raise SystemExit("sound_clips_core.c: no SOUND_CLIP_NAMES initialiser")
    names = re.findall(r'=\s*"([^"]+)"', m.group(1))
    if not names:
        raise SystemExit("sound_clips_core.c: SOUND_CLIP_NAMES extracted empty")
    return names


# Shared by every game: how Teuvo plays, as opposed to what he plays. It sits
# between the persona and the game so a game prompt never restates it. A text
# file beside the games, so tuning it is a prompt edit, not a code change.
PLAY_REGISTER = (GAMES_DIR / "register.fi.txt").read_text(encoding="utf-8").strip()


def play_instruction(persona: lp.Persona, game: Game) -> str:
    return f"{persona.instruction}\n\n{PLAY_REGISTER}\n\nLeikki: {game.title}\n{game.prompt}"


def play_tools(clips: list[str]) -> list[Any]:
    """The robot actions a play session may call. Both are NON_BLOCKING: the
    stub answers at once, and a real robot should not freeze the conversation
    while its head turns."""
    from google.genai import types

    return [
        types.Tool(
            function_declarations=[
                types.FunctionDeclaration(
                    name="look",
                    description=(
                        "Turn Teuvo's head (camera) to look somewhere. Pan: negative "
                        "is left, positive right. Tilt: negative down, positive up."
                    ),
                    behavior=types.Behavior.NON_BLOCKING,
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "pan_deg": types.Schema(
                                type=types.Type.NUMBER, minimum=-60, maximum=60
                            ),
                            "tilt_deg": types.Schema(
                                type=types.Type.NUMBER, minimum=-30, maximum=30
                            ),
                        },
                        required=["pan_deg", "tilt_deg"],
                    ),
                ),
                # Silence as an action. On 2026-11-16, proactive audio plus "if
                # nothing is new, stay silent" still drew a spoken reply to 4 of
                # 4 heartbeats, one of them about a tower that was not there.
                # The robotics streaming guide's heartbeat answers with an `ack`
                # call for the same reason.
                types.FunctionDeclaration(
                    name="wait",
                    description=(
                        "Call this instead of speaking when nothing new has happened "
                        "and the child is not waiting for you. Say nothing after it."
                    ),
                    behavior=types.Behavior.NON_BLOCKING,
                ),
                types.FunctionDeclaration(
                    name="sound",
                    description="Play one short sound effect from Teuvo's speaker.",
                    behavior=types.Behavior.NON_BLOCKING,
                    parameters=types.Schema(
                        type=types.Type.OBJECT,
                        properties={
                            "name": types.Schema(type=types.Type.STRING, enum=clips)
                        },
                        required=["name"],
                    ),
                ),
            ]
        )
    ]


def play_config(instruction: str, voice: str, clips: list[str]) -> Any:
    """Session config, built here so a test can construct it offline and catch
    an SDK signature change before a paid live run does."""
    from google.genai import types

    return types.LiveConnectConfig(
        response_modalities=["AUDIO"],
        speech_config=types.SpeechConfig(
            voice_config=types.VoiceConfig(
                prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice)
            )
        ),
        system_instruction=instruction,
        tools=play_tools(clips),
        proactivity=types.ProactivityConfig(proactive_audio=True),
        # No enable_affective_dialog: the docs list it as supported on
        # gemini-3.8-live, but on 2026-11-16 setting it closed the session with
        # 1007 "invalid argument" on both v1beta and v1alpha (bisected against
        # this config with every other field kept).
        context_window_compression=types.ContextWindowCompressionConfig(
            sliding_window=types.SlidingWindow()
        ),
        input_audio_transcription=types.AudioTranscriptionConfig(),
        output_audio_transcription=types.AudioTranscriptionConfig(),
    )


def stub_tool_result(name: str, args: dict[str, Any], clips: list[str]) -> dict:
    """What the stub robot reports back. An unknown clip or tool is an error the
    model sees, as it would be on the robot, rather than a silent success."""
    if name == "wait":
        return {"ok": True}
    if name == "look":
        return {
            "ok": True,
            "pan_deg": args.get("pan_deg"),
            "tilt_deg": args.get("tilt_deg"),
        }
    if name == "sound":
        clip = args.get("name")
        if clip in clips:
            return {"ok": True, "played": clip}
        return {"ok": False, "error": f"unknown sound {clip!r}; one of {clips}"}
    return {"ok": False, "error": f"unknown tool {name!r}"}


# --------------------------------------------------------------------------
# Timing policy (pure)
# --------------------------------------------------------------------------


@dataclass
class HalfDuplexGate:
    """Mute the mic from the first reply sample until `hangover_s` after the
    last one has played. Disabled, it always passes (headphones)."""

    hangover_s: float
    enabled: bool = True
    playing_until: float = 0.0

    def note_playback(self, now: float, queued_s: float) -> None:
        """Audio of `queued_s` seconds was queued at `now`, after what is queued."""
        start = max(now, self.playing_until)
        self.playing_until = start + queued_s

    def flush(self, now: float) -> None:
        """Playback was cut (interruption): the speaker is quiet from now."""
        self.playing_until = min(self.playing_until, now)

    def is_playing(self, now: float) -> bool:
        return now < self.playing_until

    def mic_open(self, now: float) -> bool:
        return not self.enabled or now >= self.playing_until + self.hangover_s


def heartbeat_due(
    now: float,
    last_heartbeat: float,
    last_sound: float,
    playing: bool,
    interval_s: float,
    quiet_s: float,
) -> bool:
    """Send the next heartbeat only into a quiet session.

    `last_sound` is the later of the child's last transcribed speech and
    Teuvo's last reply audio. A heartbeat sent while either is talking would
    barge in: the docs warn that a prompt during generation cancels it.
    """
    if playing:
        return False
    return now - last_heartbeat >= interval_s and now - last_sound >= quiet_s


def split_jpegs(buf: bytes) -> tuple[list[bytes], bytes]:
    """Cut complete JPEGs out of an MJPEG byte stream; return them and the rest.

    ffmpeg's image2pipe writes back-to-back JPEGs with no framing. Bytes before
    the first SOI are dropped; an unfinished frame is returned as the rest.
    """
    frames: list[bytes] = []
    pos = 0
    while True:
        start = buf.find(JPEG_SOI, pos)
        if start < 0:
            return frames, b""
        end = buf.find(JPEG_EOI, start + 2)
        if end < 0:
            return frames, buf[start:]
        frames.append(buf[start : end + 2])
        pos = end + 2


# --------------------------------------------------------------------------
# The session log (pure apart from the final write)
# --------------------------------------------------------------------------


@dataclass
class LogEvent:
    t: float  # seconds since the session started
    kind: str
    data: Any = None


@dataclass
class SessionLog:
    t0: float
    events: list[LogEvent] = field(default_factory=list)

    def add(self, now: float, kind: str, data: Any = None) -> None:
        self.events.append(LogEvent(round(now - self.t0, 3), kind, data))


def _merge_runs(events: list[LogEvent]) -> list[LogEvent]:
    """Join consecutive transcription fragments of one speaker into one line."""
    out: list[LogEvent] = []
    for e in events:
        if out and e.kind in ("child", "teuvo") and out[-1].kind == e.kind:
            out[-1] = LogEvent(out[-1].t, e.kind, out[-1].data + e.data)
        else:
            out.append(LogEvent(e.t, e.kind, e.data))
    return out


def transcript_markdown(game: Game, model: str, events: list[LogEvent]) -> str:
    """A reviewable script of the session: who said what, when, and what the
    robot would have done. The thing to read after a run."""
    lines = [f"# {game.title} — {model}"]
    for e in _merge_runs(events):
        stamp = f"`{int(e.t // 60):02d}:{e.t % 60:04.1f}`"
        if e.kind == "child":
            lines.append(f"{stamp} **Lapsi:** {e.data.strip()}")
        elif e.kind == "teuvo":
            lines.append(f"{stamp} **Teuvo:** {e.data.strip()}")
        elif e.kind == "heartbeat":
            lines.append(f"{stamp} _heartbeat (frame {e.data})_")
        elif e.kind == "tool_call" and e.data["name"] == "wait":
            lines.append(f"{stamp} _(Teuvo waits)_")
        elif e.kind == "tool_call":
            lines.append(
                f"{stamp} 🔧 `{e.data['name']}({json.dumps(e.data['args'], ensure_ascii=False)})`"
            )
        elif e.kind == "interrupted":
            lines.append(f"{stamp} _interrupted_")
        elif e.kind == "go_away":
            lines.append(f"{stamp} _GoAway: {e.data}_")
        elif e.kind == "end":
            lines.append(f"{stamp} _end: {e.data}_")
    return "\n\n".join(lines) + "\n"


def session_summary(events: list[LogEvent]) -> dict[str, Any]:
    kinds = [e.kind for e in events]
    usage = [e.data for e in events if e.kind == "usage"]
    end = next((e for e in reversed(events) if e.kind == "end"), None)
    return {
        "duration_s": end.t if end else (events[-1].t if events else 0.0),
        "end_reason": end.data if end else None,
        "heartbeats": kinds.count("heartbeat"),
        "frames_sent": kinds.count("frame"),
        "teuvo_turns": kinds.count("turn_complete"),
        "interruptions": kinds.count("interrupted"),
        "tool_calls": kinds.count("tool_call"),
        "usage_last": usage[-1] if usage else None,
    }


# --------------------------------------------------------------------------
# Devices and the live session (network + hardware; not unit-tested)
# --------------------------------------------------------------------------


async def camera_frames(
    device: str, width: int, on_frame: Callable[[bytes], None], stop: asyncio.Event
) -> None:
    """Grab 1 frame/s from an AVFoundation camera through ffmpeg (no OpenCV)."""
    proc = await asyncio.create_subprocess_exec(
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "avfoundation", "-framerate", "30", "-pixel_format", "nv12",
        "-i", f"{device}:none",
        "-vf", f"fps={1 / FRAME_INTERVAL_S},scale={width}:-2",
        "-c:v", "mjpeg", "-q:v", "5", "-f", "image2pipe", "-",
        stdout=asyncio.subprocess.PIPE,
    )  # fmt: skip
    assert proc.stdout is not None
    rest = b""
    try:
        while not stop.is_set():
            chunk = await proc.stdout.read(65536)
            if not chunk:
                raise RuntimeError(
                    f"ffmpeg camera {device!r} exited ({proc.returncode})"
                )
            frames, rest = split_jpegs(rest + chunk)
            for f in frames:
                on_frame(f)
    finally:
        with contextlib.suppress(ProcessLookupError):
            proc.terminate()
        await proc.wait()


class Speaker:
    """24 kHz output through sounddevice, fed from a byte queue the gate mirrors."""

    def __init__(self, sd: Any, device: int | None) -> None:
        self._buf = bytearray()
        self._stream = sd.RawOutputStream(
            samplerate=OUTPUT_RATE, channels=1, dtype="int16",
            device=device, callback=self._cb,
        )  # fmt: skip

    def _cb(self, outdata, frames, _time, _status) -> None:
        n = frames * 2
        chunk = bytes(self._buf[:n])
        del self._buf[:n]
        outdata[: len(chunk)] = chunk
        outdata[len(chunk) :] = b"\x00" * (n - len(chunk))

    def write(self, pcm: bytes) -> None:
        self._buf.extend(pcm)

    def flush(self) -> None:
        self._buf.clear()

    def __enter__(self) -> Speaker:
        self._stream.start()
        return self

    def __exit__(self, *exc) -> None:
        self._stream.stop()
        self._stream.close()


async def run_play(args: argparse.Namespace, api_key: str, model: str) -> int:
    import sounddevice as sd
    from google import genai
    from google.genai import types

    game_path: Path = args.game
    game = load_game(
        game_path.read_text(encoding="utf-8"), game_path.name.split(".")[0]
    )
    persona = lp.extract_persona(lp.PERSONA_C.read_text(encoding="utf-8"))
    clips = sound_clip_names(SOUND_CLIPS_C.read_text(encoding="utf-8"))
    voice = args.voice or persona.voice
    instruction = play_instruction(persona, game)
    config = play_config(instruction, voice, clips)

    stamp = dt.datetime.now(dt.UTC).strftime("%Y%m%dT%H%M%SZ")
    out = args.out_root / f"{stamp}-{game.slug}"
    (out / "frames").mkdir(parents=True, exist_ok=True)
    (out / "instruction.txt").write_text(instruction + "\n", encoding="utf-8")

    loop = asyncio.get_running_loop()
    t0 = time.monotonic()
    log = SessionLog(t0)
    gate = HalfDuplexGate(args.hangover_ms / 1000, enabled=not args.barge_in)
    mic_q: asyncio.Queue[bytes] = asyncio.Queue()
    stop = asyncio.Event()
    state = {"frame": None, "frame_no": 0, "last_sound": t0, "last_hb": t0}
    teuvo_pcm, mic_pcm = bytearray(), bytearray()

    def on_mic(indata, _frames, _time, _status) -> None:
        loop.call_soon_threadsafe(mic_q.put_nowait, bytes(indata))

    def on_frame(jpeg: bytes) -> None:
        state["frame"] = jpeg

    client = genai.Client(api_key=api_key)
    print(f"model={model} voice={voice} game={game.slug} → {out}", file=sys.stderr)
    end_reason = "unknown"
    with (
        Speaker(sd, args.speaker) as speaker,
        sd.RawInputStream(
            samplerate=INPUT_RATE, channels=1, dtype="int16", device=args.mic,
            blocksize=INPUT_RATE * MIC_BLOCK_MS // 1000, callback=on_mic,
        ),
    ):  # fmt: skip
        async with client.aio.live.connect(model=model, config=config) as session:
            log.add(time.monotonic(), "connected", model)

            async def send_mic() -> None:
                mime = f"audio/pcm;rate={INPUT_RATE}"
                while not stop.is_set():
                    block = await mic_q.get()
                    if gate.mic_open(time.monotonic()):
                        mic_pcm.extend(block)
                        await session.send_realtime_input(
                            audio=types.Blob(data=block, mime_type=mime)
                        )

            async def send_frames() -> None:
                while not stop.is_set():
                    await asyncio.sleep(FRAME_INTERVAL_S)
                    jpeg = state["frame"]
                    if jpeg is None:
                        continue
                    state["frame_no"] += 1
                    n = state["frame_no"]
                    (out / "frames" / f"{n:05d}.jpg").write_bytes(jpeg)
                    log.add(time.monotonic(), "frame", n)
                    await session.send_realtime_input(
                        video=types.Blob(data=jpeg, mime_type="image/jpeg")
                    )
                    now = time.monotonic()
                    if heartbeat_due(
                        now, state["last_hb"], state["last_sound"],
                        gate.is_playing(now), args.heartbeat_s, args.quiet_s,
                    ):  # fmt: skip
                        state["last_hb"] = now
                        log.add(now, "heartbeat", n)
                        await session.send_realtime_input(text=game.heartbeat)

            async def receive() -> None:
                nonlocal end_reason
                while not stop.is_set():
                    async for msg in session.receive():
                        now = time.monotonic()
                        sc = msg.server_content
                        if sc is not None:
                            for part in (
                                sc.model_turn.parts if sc.model_turn else None
                            ) or []:
                                blob = part.inline_data
                                if blob is not None and blob.data:
                                    rate = lp.parse_pcm_rate(blob.mime_type)
                                    if rate != OUTPUT_RATE:
                                        raise RuntimeError(
                                            f"unexpected reply format {blob.mime_type!r}"
                                        )
                                    speaker.write(blob.data)
                                    teuvo_pcm.extend(blob.data)
                                    gate.note_playback(
                                        now, len(blob.data) / 2 / OUTPUT_RATE
                                    )
                                    state["last_sound"] = max(
                                        state["last_sound"], gate.playing_until
                                    )
                            if sc.input_transcription and sc.input_transcription.text:
                                state["last_sound"] = now
                                log.add(now, "child", sc.input_transcription.text)
                            if sc.output_transcription and sc.output_transcription.text:
                                log.add(now, "teuvo", sc.output_transcription.text)
                            if sc.interrupted:
                                speaker.flush()
                                gate.flush(now)
                                log.add(now, "interrupted")
                            if sc.turn_complete:
                                log.add(now, "turn_complete")
                        if msg.tool_call is not None:
                            responses = []
                            for call in msg.tool_call.function_calls or []:
                                call_args = dict(call.args or {})
                                result = stub_tool_result(call.name, call_args, clips)
                                log.add(
                                    now,
                                    "tool_call",
                                    {
                                        "name": call.name,
                                        "args": call_args,
                                        "result": result,
                                    },
                                )
                                print(
                                    f"  🔧 {call.name}({call_args}) → {result}",
                                    file=sys.stderr,
                                )
                                responses.append(
                                    types.FunctionResponse(
                                        id=call.id, name=call.name, response=result,
                                        scheduling=types.FunctionResponseScheduling.SILENT,
                                    )
                                )  # fmt: skip
                            await session.send_tool_response(
                                function_responses=responses
                            )
                        if msg.usage_metadata is not None:
                            log.add(
                                now,
                                "usage",
                                msg.usage_metadata.model_dump(
                                    exclude_none=True, mode="json"
                                ),
                            )
                        if msg.go_away is not None:
                            log.add(now, "go_away", str(msg.go_away.time_left))
                            end_reason = "go_away"
                            stop.set()
                            return

            async def deadline() -> None:
                nonlocal end_reason
                await asyncio.sleep(args.max_minutes * 60)
                end_reason = "max_minutes"
                stop.set()

            async def opening() -> None:
                # Wait for the first frame so Teuvo's greeting can be about the scene.
                for _ in range(50):
                    if state["frame"] is not None:
                        break
                    await asyncio.sleep(0.1)
                await asyncio.sleep(FRAME_INTERVAL_S * 1.5)
                state["last_hb"] = time.monotonic()
                log.add(state["last_hb"], "heartbeat", state["frame_no"])
                await session.send_realtime_input(text=game.opening)
                await stop.wait()  # finishing early would read as "session over"

            tasks = [
                asyncio.create_task(c)
                for c in (
                    camera_frames(args.camera, args.frame_width, on_frame, stop),
                    send_mic(), send_frames(), receive(), deadline(), opening(),
                )
            ]  # fmt: skip
            stopper = asyncio.create_task(stop.wait())
            try:
                done, _ = await asyncio.wait(
                    [stopper, *tasks], return_when=asyncio.FIRST_COMPLETED
                )
                for t in done - {stopper}:
                    if t.exception() is not None:
                        end_reason = f"error: {t.exception()!r}"
            except asyncio.CancelledError:
                end_reason = "ctrl-c"
            finally:
                stop.set()
                for t in [*tasks, stopper]:
                    t.cancel()
                await asyncio.gather(*tasks, stopper, return_exceptions=True)

    log.add(time.monotonic(), "end", end_reason)
    with (out / "events.jsonl").open("w", encoding="utf-8") as fh:
        for e in log.events:
            fh.write(
                json.dumps(
                    {"t": e.t, "kind": e.kind, "data": e.data}, ensure_ascii=False
                )
                + "\n"
            )
    (out / "transcript.md").write_text(
        transcript_markdown(game, model, log.events), encoding="utf-8"
    )
    summary = {
        "model": model,
        "voice": voice,
        "game": game.slug,
        **session_summary(log.events),
    }
    (out / "session.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (out / "teuvo.wav").write_bytes(lp.wav_bytes(bytes(teuvo_pcm), OUTPUT_RATE))
    (out / "mic.wav").write_bytes(lp.wav_bytes(bytes(mic_pcm), INPUT_RATE))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"transcript: {out / 'transcript.md'}", file=sys.stderr)
    return 0 if not end_reason.startswith("error") else 1


def list_devices() -> int:
    import subprocess

    import sounddevice as sd

    print("Audio (sounddevice index, for --mic / --speaker):")
    print(sd.query_devices())
    print("\nCameras (AVFoundation index, for --camera):")
    res = subprocess.run(
        ["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
        capture_output=True, text=True,
    )  # fmt: skip
    in_video = False
    for line in res.stderr.splitlines():
        if "video devices" in line:
            in_video = True
        elif "audio devices" in line:
            in_video = False
        elif in_video and "] [" in line:
            print("  " + line.split("] ", 1)[1])
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("out_root", nargs="?", type=Path)
    p.add_argument("--game", type=Path, default=GAMES_DIR / "building.fi.toml")
    p.add_argument("--model", help="Live model id; must appear in ListModels")
    p.add_argument("--voice", help="prebuilt voice (default: the persona's own)")
    p.add_argument("--camera", default="0", help="AVFoundation video index or name")
    p.add_argument("--frame-width", type=int, default=768)
    p.add_argument("--mic", type=int, default=None, help="sounddevice input index")
    p.add_argument("--speaker", type=int, default=None, help="sounddevice output index")
    p.add_argument("--heartbeat-s", type=float, default=DEFAULT_HEARTBEAT_S)
    p.add_argument("--quiet-s", type=float, default=DEFAULT_QUIET_S)
    p.add_argument("--hangover-ms", type=int, default=DEFAULT_HANGOVER_MS)
    p.add_argument(
        "--barge-in", action="store_true", help="never mute the mic (headphones)"
    )
    p.add_argument("--max-minutes", type=float, default=DEFAULT_MAX_MINUTES)
    p.add_argument("--list-devices", action="store_true")
    args = p.parse_args(argv)

    if args.list_devices:
        return list_devices()
    if args.out_root is None:
        p.error("out_root is required unless --list-devices is given")
    api_key = os.environ.get("GEMINI_API_KEY", "")
    if not api_key:
        print(
            "GEMINI_API_KEY is not set (expected from ~/.api_tokens via mise)",
            file=sys.stderr,
        )
        return 2
    try:
        model = lp.choose_model(
            lp.live_models(lp.fetch_model_listing(api_key)), args.model
        )
    except ValueError as err:
        print(f"model selection: {err}", file=sys.stderr)
        return 2
    try:
        return asyncio.run(run_play(args, api_key, model))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
