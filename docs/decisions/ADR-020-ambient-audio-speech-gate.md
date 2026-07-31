# ADR-020: Ambient Audio as a Second Evidence Gate on Speech (OR, not AND)

**Status**: accepted
**Date**: 2026-07-31
**Source**: conversation 2026-07-30/31
**Confidence**: 7/10

---

## Context

The XIAO ESP32-S3 Sense carries an onboard PDM microphone (GPIO42 CLK / GPIO41
DATA) that this firmware had never used. It costs no header pins and no wiring —
unusual for this board, where the GPIO budget has been exhausted since ADR-019.

`robocar-unified` already decides on-device whether the planner is even *offered*
the `speak` tool, because every Gemini request is stateless: the model has no
previous frame and no record of having spoken, so an instruction like "remark
only when the scene changes" names a fact it cannot check. Two gates existed:

- **`speech_budget`** — a ration. Minimum gap plus a rolling per-window cap.
- **`scene_change`** — evidence. Has the *view* changed since the robot last
  spoke?

The failure that motivated a third gate is the mirror of the one `scene_change`
fixed. Pointed at a static bench, the robot used to narrate the same unmoving
thing every 15 s; `scene_change` fixed that. But a robot that can only see has
the opposite blind spot: a person can walk into a still room and speak to it, and
nothing in its evidence changes at all.

## Decision

Add `ambient_audio` — a pure, host-testable gate over continuously sampled
microphone frames — and combine the three gates as:

```c
speech_budget_allows(now_ms) && (scene_change_novel() || ambient_audio_novel(now_ms))
```

**A ration ANDs; two senses OR.** The budget caps how much the robot may talk at
all, and a ration that evidence can vote down is not a ration. Scene and ambient
answer the *same* question — is there anything new to remark on? — through
different senses, so they combine disjunctively.

The gate measures two orthogonal, level-shift-invariant quantities, both relative
rather than absolute:

- **Loudness excursion** above a noise floor that falls quickly and rises at a
  bounded rate, so a newly started steady sound (a fan) is absorbed within
  seconds and stops being news *by construction*.
- **Spectral-shape distance**, with each band stored as its deviation from the
  mean of the bands, so a uniform gain change cancels exactly.

Scores are **latched peaks** since the robot last spoke, not instantaneous
readings, because the microphone is sampled ~200x more often than the planner
runs.

## Why OR rather than AND

1. **The decisive case fails under AND.** Someone enters a still room and speaks.
   The soundscape changed; the view may not have (they entered at the frame edge,
   or stood behind the camera). Under AND the robot is mute exactly when a human
   has just addressed it — which reads as broken, not as laconic.
2. **Conjoining two conservative detectors multiplies their miss rates.** Each
   was tuned to be cautious alone; ANDing them yields a gate far more
   conservative than either author intended, and neither threshold can fix it.
3. **The error costs are asymmetric.** A false positive is one unwanted sentence,
   hard-capped by the budget at three per five minutes. A false negative is the
   robot ignoring a person, uncapped and unsignalled.
4. **Sampling rates differ by ~200x.** A conjunction of a continuously armed
   detector with a sparsely sampled one is dominated by the sparse one, so AND
   would be approximately "scene only, minus whatever audio subtracts" — the
   audio gate would spend its existence removing opportunities and never adding
   any.
5. **The worst case is bounded and recoverable.** Under OR it is spending the
   full ration, tunable from the console in seconds. Under AND it is silence in a
   live room, indistinguishable from a hardware fault.

## Consequences

**The prompt had to change too, and this is the subtle part.** Opening the tool
is only half the mechanism. The speech clause read *"do so only if this frame
shows something worth remarking on"* — correct while the only evidence is visual,
and wrong the moment audio can open the gate, because on an audio-only cycle the
frame shows nothing new *by construction*. Left alone, the prompt would have
instructed the model to stay silent in precisely the case the new gate existed to
permit, and the audio branch would have produced silence or a remark about the
wrong thing. `build_request_json()` now names the evidence and, on an audio-only
opening, tells the model to remark on what it heard rather than what it sees.
This is the same law the whole subsystem rests on: the request is stateless, so
anything the model must know has to be put into it.

**Both references move together.** `ambient_audio_mark_spoken()` is called beside
`scene_change_mark_spoken()`; marking only one would leave the other licensing
remarks about evidence already used.

**The robot is now deaf-and-blind-quiet rather than blind-quiet.** In a static,
silent room it says nothing, which remains correct. The `mic` console command and
the per-cycle `gate:` log field exist so that state is distinguishable from a
fault.

**The decision is auditable rather than merely argued.** Every planner cycle logs
each score beside its own threshold plus which branch decided (`-`/`V`/`A`/`VA`/
`.`). If `A`-only cycles track the building ventilation, the thresholds are
wrong; if `A` never appears, the gate is dead weight. Both verdicts come from one
20-minute capture.

**The thresholds are console knobs, not constants** (`voice loud`, `voice
sound`), for the reason `cam gainceiling` is: whether the robot is pleasantly
laconic or annoyingly mute needs somebody in the room. They do not persist to
NVS. Note the inverted zero convention — `0` disables an ORed sub-gate, whereas
`voice scene 0` makes the ANDed scene gate always-novel. Both mean "not
participating"; the polarity follows the operator, and a host test pins that both
audio thresholds at 0 restores pre-microphone behaviour exactly.

## Alternatives considered

- **AND with `scene_change`.** Rejected above.
- **A wake word (esp-sr WakeNet/MultiNet).** The 8 MB partition table is full
  (2 x 3.5 MB OTA + 960 kB SPIFFS) with no room for a model partition, MultiNet
  has no Finnish, and custom wake words need a paid training service. Revisit
  only if the robot must respond with WiFi down.
- **Sampling audio inline in the planner loop.** 64 ms every 15 s observes 0.4%
  of elapsed time; almost every transient would fall in the gap. This is why
  `ambient_listener` is a task.

## Implementation notes

PDM RX exists **only on I2S0** on the ESP32-S3, which `audio_player.c` already
holds for the amplifier. The RX channel must therefore be allocated by a
**separate, RX-only** `i2s_new_channel()` call: one call requesting both handles
sets `controller->full_duplex = true` (`i2s_common.c:998`), and the driver then
shares BCLK/WS and forces the mic to slave (`i2s_pdm.c:389,397`) — a 16 kHz
microphone clocked off a 24 kHz amplifier that also dies whenever the amp powers
down. The reasoning is recorded at the allocation site in `mic_pdm.c`, because
merging the two calls looks like an obvious simplification.

`mic_dump.c` and `tools/decode-mic-dump.py` ship with the gate rather than as a
follow-up, for the reason `frame_dump.c` did: every claim about the camera was
inference until someone looked at a frame, and the first one dumped disproved the
sensor named in the header. A loudness threshold over a microphone nobody has
listened to is guesswork, and it will present itself as a tuning problem.

## Related

- [ADR-016](ADR-016-hierarchical-ai-controller.md) — the planner/executor split
- [ADR-019](ADR-019-robocar-voice-gemini-tts.md) — the voice output path this gates
- `.claude/rules/stateless-model-gating.md` — withhold the capability, don't word
  the instruction harder
- `.claude/rules/camera-sensor-identity.md` — the sibling "read the sensor before
  reasoning about it" lesson
