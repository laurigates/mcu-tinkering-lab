# A Working Circuit Is Not a Correctly Connected One

A chip whose **GND is not connected still runs.** It finds a return path through
the ESD clamp diodes on its input pins, and that path is high-impedance and
signal-dependent — so the part does not fail cleanly, it works *badly*, in a way
that reads as a firmware bug. Meanwhile the driving GPIOs carry current their
clamp structures were never rated for.

This is the hardware sibling of `esp-idf-rmt-rx.md`'s meta-lesson: a
degraded-but-working fallback path masks the fault it is standing in for.

> **Canonical break (2026-08-26, robocar-unified):** the MAX98357A's ground wire
> was disconnected. The amp kept playing, because its supply current returned
> through the ESD diodes on BCLK, LRCLK and DIN into the XIAO's GPIO drivers.
> It presented as *"much more silent and a bit distorted"*, and got worse as more
> of the wiring harness was inserted — each board half degraded it, both together
> compounded. Two full diagnoses were built and discarded first: leakage shifting
> the `GAIN_SLOT` bracket, then supply droop. The actual fault was a wire that
> was never in the circuit. Reconnecting it restored clear, loud audio.

## The three symptoms that name this fault

Together these are close to diagnostic, and none of them is what a firmware bug
looks like:

| Symptom | Why a phantom return produces it |
|---|---|
| Quieter, not noisier | The chip's local ground floats **up** under load, so actual VDD-to-GND across the die is less than the rail |
| Distortion that scales with signal | The float is proportional to instantaneous current, so it modulates with the audio |
| **Compounds with harness length** | More wiring = longer, higher-impedance parasitic return. Direct cables are clean |

**Check continuity to ground on every supposedly-grounded pin before believing
any downstream theory.** It costs one continuity beep. Both discarded diagnoses
above would have died instantly to that measurement.

**Then check the pins that were carrying the return.** They were conducting
outside their rating for however long the fault existed. Verify amplitude and
edge shape on each (X10 probe), rather than assuming they survived.

## "Quieter" and "noisier" are different diagnoses — do not blur them

Digital corruption of a serial audio stream (a dropped byte, a misaligned
sample, a torn DMA buffer) **adds noise; it does not reduce level.** A level drop
therefore redirects the search away from the transport entirely, before any
instrument comes out.

Applied here, it is what ruled out the whole class the previous month of work had
been chasing — the misalignment bug, the ring-buffer accounting, the streaming
gate. None of those can make a robot quieter.

## Measure the effect, not the high-impedance node

Analog configuration pins are *sense nodes*, not logic inputs, and a probe on one
is part of the circuit. The MAX98357A picks its gain by comparing `GAIN_SLOT`
against **fractions of VDD** (12 dB at 0–0.10×, 15 dB at 0.15–0.35×, 9 dB at
0.40–0.60×, 3 dB at 0.65–0.85×, 6 dB at 0.90×–VDD, with undefined gaps between),
so "floating = 9 dB" means *parked at mid-rail by an internal divider* — the most
leakage-sensitive state the pin has, and one a 1 MΩ scope input visibly moves.

So prefer a measurement that does not touch the node: **play a fixed, locally
generated stimulus through the shipped path and compare the output between
configurations.** `robocar-bringup`'s `amp` check exists for this — a synthesised
440 Hz / 880 Hz / 200 Hz–2 kHz sweep, no network, no API key, generated far
faster than real time so the ring cannot underrun and the DMA cannot tear. A
ratio of 0.50 between two configurations is a 6 dB gain-bracket change; ~1.0 with
flattened peaks is clipping; ~1.0 and clean exonerates the amplifier.

Corollary for class-D specifically: **do not read Vpp at the speaker terminals.**
A filterless output is a ~300 kHz PWM square wave swinging the full rail at any
volume — the datasheet says it relies on the speaker's inductance and the ear to
demodulate. Vpp reads ≈2×VDD regardless of level. Use true-RMS AC volts in-band,
or an RC low-pass ahead of the probe.

## Confirm the instrument can see the thing before trusting a negative

An instrument that cannot resolve the phenomenon returns a confident, wrong
"fine". Check the spec, not the reading. From `docs/zt-703s/`:

| Want to measure | Trap | Do instead |
|---|---|---|
| Supply droop during audio | The DMM samples **3×/second**; Min-hold cannot see a millisecond sag and reports a healthy rail | Scope, DC-coupled, Scroll Mode ≥200 ms/div, infinite persistence, Vmin |
| A mid-rail sense pin | Scope input is **1 MΩ / 16 pF**; at X1 it drags the node into another bracket | X10 (10 MΩ), and compare the *delta* between configurations, not the absolute |
| Drop across a ground wire | Both channels **share ground**; clipping the probe there shorts out the thing being measured | Two single-ended passes, each referenced to its own local ground |

## Read the datasheet; do not recall it

Every number that moved this diagnosis came from the actual document, and two of
them **closed off** lines of inquiry rather than opening them:

- Speaker-side wire resistance was ruled out quantitatively — the datasheet's own
  worked example puts 100 mΩ of speaker trace at ~0.2 dB.
- Supply ripple was ruled out — PSRR is 77 dB at 217 Hz, so caps address droop
  and UVLO (1.4–2.3 V), not noise.
- Output power scales with V², not gain: 8 Ω at 1% THD is 1.4 W at 5 V and
  0.77 W at 3.7 V. Droop costs **headroom**, so it explains clipping, never a
  quiet signal that is otherwise clean.

## When it bites

- Any board where the ground is a **connector or harness hop** rather than a
  plane — which is every project split across a main board and a module board.
- Symptoms that **change when you flex, reseat, or extend the wiring**. That is
  a connection fault, not a code path.
- Any moment two competing electrical theories both fit. Both fitting is a
  signal that the real fault is upstream of both.

## Related

- `esp-idf-rmt-rx.md` — a fallback path that "works" while hiding the primary
  path's bug; same shape, digital domain
- `camera-sensor-identity.md` — never render an unread sensor as a measurement;
  here, never trust a reading from an instrument that cannot resolve it
- `~/.claude/rules/diagnose-at-the-failure-point.md` — read the actual resource
  at the failure point rather than reasoning from the framing you inherited
- `docs/zt-703s/ZT-703S_User_Manual.md` — the bench instrument's limits, which
  are part of the methodology and not just its controls
