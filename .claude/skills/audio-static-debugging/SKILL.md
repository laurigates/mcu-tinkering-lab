---
name: audio-static-debugging
description: Debug static, hiss, or noise on embedded I2S audio (ESP32 + MAX98357A / PCM DACs), especially streamed PCM. Measure the signal at each pipeline stage instead of guessing at the amp. Use when audio plays but sounds wrong.
user-invocable: true
argument-hint: "[symptom: static before speech / during / throughout]"
allowed-tools: Bash, Read, Grep, Glob
---

# Debugging Embedded Audio Static / Streaming-PCM Noise

Static on an I2S speaker (MAX98357A, PCM510x, etc.) has several distinct causes
that sound similar to the human reporting them, and the wrong ones are seductive
because they're about *hardware* — the amp, the clock, the ground. This skill
exists because in the robocar-unified voice bring-up (2026-07) **two hardware
theories were wrong** (uninitialised DMA content; BCLK stop/start pops) and cost
several reflash cycles each. The fix only came from **measuring the actual
samples** at each stage. Measure first; theorise last.

## The one law

**You cannot hear the audio; the user can, and the samples can.** Do not reason
about the amp/clock/DMA from the symptom word "static." Get the PCM bytes and
compute statistics on them — that localises the corruption to a pipeline stage
in one pass, with no hardware and no guessing.

## Step 1 — get a precise symptom from the user (it discriminates the cause)

"Terrible static" underdetermines everything. Ask which:

| Symptom | Most likely stage |
|---|---|
| Static **before/after** the line, silence-adjacent | Amp idle / clock-gating / floating DIN (analog) |
| Speech **intelligible but with static bursts**, or alternating clean↔noise | **Sample misalignment** in the transport (see Step 3) |
| Speech **fully garbled**, no words | Format mismatch: endianness, bit width, or sample rate |
| Faint constant hiss under clean speech | Amp noise floor / gain / power rail |

## Step 2 — prove whether the *source* data is clean (before blaming the pipeline)

Capture the PCM the device receives (e.g. the base64 `inlineData` from a TTS
response, or a dump from the decode buffer) and characterise it. Clean speech
and byte-swapped/garbage have unmistakably different statistics:

```sh
python3 - <<'PY'
import json, base64, struct, math
raw = base64.b64decode(json.load(open("resp.json"))
        ["candidates"][0]["content"]["parts"][0]["inlineData"]["data"])
n = len(raw)//2
def stats(fmt, label):
    s = struct.unpack(("<" if fmt=="LE" else ">")+"%dh"%n, raw[:n*2])
    rms = math.sqrt(sum(x*x for x in s)/n)
    big = sum(1 for x in s if abs(x) > 16384)/n            # fraction past half-scale
    zc  = sum(1 for i in range(1,n) if (s[i-1]>=0)!=(s[i]>=0))/(n-1)  # zero-cross rate
    print(f"{label}: RMS={rms:7.0f}  >half-scale={big*100:4.1f}%  zero-cross={zc*100:4.1f}%")
stats("LE","little-endian"); stats("BE","big-endian")
PY
```

- **Clean speech**: low zero-cross rate (~5–15%), few samples past half-scale
  (~1–3%), moderate RMS. The interpretation that yields this is the correct one.
- **Byte-swapped / garbage**: zero-cross ~40–50% (near-random), ~40% past
  half-scale, high RMS.

Real finding: Gemini TTS advertises `audio/L16` (nominally **big-endian** per
RFC) but actually returns **little-endian** — the LE stats were clean, BE was
noise. **Verify endianness from the data before swapping bytes**; a "fix" based
on the label would have broken working audio. If the source is clean, the
corruption is downstream — go to Step 3.

## Step 3 — the streaming-PCM alignment trap (the robocar root cause)

**16-bit samples must never be split across a transport boundary.** The classic
break, and the one that produced "speech + terrible static, alternating":

- A FreeRTOS `RINGBUF_TYPE_BYTEBUF` (byte-granular) carries the PCM.
- The producer feeds it **odd-length** chunks — a base64 decoder emits **3-byte**
  groups per quartet, and 3 is odd.
- The consumer reads `got` bytes and does `samples = got / sizeof(int16_t)`, then
  returns the whole item — so an **odd `got` drops the straggler byte**.
- That drop shifts every subsequent sample by one byte. The shift is
  **cumulative**: after an even number of drops the stream is re-aligned (clean),
  after an odd number it's misaligned (static) — hence the audio oscillates
  between clean speech and loud static, which is the tell.

**Fix — keep the ring 16-bit aligned end to end.** Hold back an odd trailing
byte in the writer and prepend it to the next write, so only even-length runs
enter the ring (splice the carried byte + the first new byte into one 2-byte
sample, send the even remainder, carry any new odd tail). Reset the carry at
utterance end/abort so a half-sample can't bleed into the next utterance. With
even writes into an even-sized buffer and even-sized reads, alignment holds
through wraps. See `packages/robocar/unified/main/audio_player.c`
(`audio_player_write`) for the reference implementation.

Related sizing gotcha found the same day: the speech text buffer
(`SPEECH_TEXT_MAX`) must fit the *rendered* sentence — Finnish words are long, so
a 25-word line ran ~250 chars and a 160-char buffer truncated both text and
audio mid-word. Size for the language.

## Step 3b — starvation: the network can be slower than real time (2026-07)

Distinct from misalignment, and the cause of the *second* robocar voice bug. A
streamed source can produce audio **more slowly than the DAC consumes it**, and
the resulting underrun does not sound like a gap — it sounds like broadband
noise. Two facts, both verified against ESP-IDF v5.4 source:

- **Measure the real-time factor before theorising.** Across seven live Gemini
  TTS captures the RTF (audio-ms produced per wallclock-ms after first byte)
  was `0.55 / 0.79 / 0.84 / 1.88 / 2.70 / 3.08 / 3.81` — **three of seven below
  1.0**. The slow ones were all the *short* lines: a fixed 0.9–2.7 s
  time-to-first-byte amortised over less audio. Log an `rtf=` figure per
  utterance; below 1.00 is normal and must be *absorbed*, not prevented.
- **Underrun tears, it does not merely gap.** `i2s_common.c` creates
  `msg_queue` with depth `desc_num - 1` and, when it is full, the TX ISR
  **drops the oldest entry** (`xQueueIsQueueFullFromISR` →
  `xQueueReceiveFromISR` into a dummy) before pushing the finished descriptor.
  After a writer stall longer than `desc_num-1` descriptor periods,
  `i2s_channel_write()` is handed the *oldest* free descriptor — the very next
  one the DMA will transmit — so its `memcpy` lands inside a buffer being read
  out. `auto_clear` zeroing the buffer after send does not save you; that is
  why the symptom is noise rather than silence.

**Fix shape**: gate playback on a preroll — start when N ms are banked **OR**
the fetch completes, whichever comes first. The OR is the load-bearing half: a
short utterance (the class that starves) then plays from a complete buffer and
cannot underrun at all. Make the gate a property of the **ring**, not of an
utterance, or back-to-back speech inherits an already-open gate and skips it.
Widen `dma_desc_num` for margin, and batch the producer's writes — a decoder
emitting 3 bytes per call forces a `portYIELD_WITHIN_API()` to the
higher-priority player on every ring send (~25 000/s), starving the producer
exactly when it needs to get ahead.

## Step 4 — only now consider the analog side

If the source is clean AND the transport is aligned AND it still hisses/pops in
the **silence around** speech, it's the amp: the MAX98357A has no shutdown GPIO
on this board (budget full), so it amplifies continuously; gating BCLK / letting
DIN float between utterances pops. Options, in order: drive continuous zeros
(keep the channel enabled, feed silence when idle — trades pop for a possible
faint hiss), or gate the amp's SD pin via a freed/expander GPIO. Judge by ear —
this is the one part only the user can settle.

## Rationale

Every wrong turn in this bug was a *hardware* theory reasoned from the word
"static"; every correct step was a *measurement* of the bytes. The statistics
test in Step 2 costs one `python3` run and tells you which stage is lying. This
is `diagnose-at-the-failure-point` for audio: the samples at each stage are the
authoritative source, above any theory about the amp.
