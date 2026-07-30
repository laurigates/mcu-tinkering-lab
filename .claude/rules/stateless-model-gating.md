# Gating a Stateless Model: Withhold the Capability, Don't Word the Instruction Harder

Every `generateContent` call from this repo is **stateless** — one request, one
image, no memory of the previous call. That is easy to state and easy to forget,
and forgetting it produces a specific, recurring bug: a prompt that asks the
model to make a judgement it has no information to make. The instruction reads
perfectly. It cannot be obeyed, and nothing reports that.

Distilled from the robocar-unified voice work (2026-07, PRs #442 → #452), where
the same mistake was made three times in three different wordings before the
shape became clear.

## 1. If the model cannot check the fact, remove the tool instead

The planner prompt said, in successive attempts:

- *"Use it sparingly — only when the scene changes in a way worth remarking on."*
- *"...not on every frame."*
- *"You are consulted about every 15 seconds, so narrating every time would be
  tiresome."*

All three name facts the model **structurally cannot access**: it has never seen
the previous frame, and it has no record of having spoken. So it behaved like a
first-time observer on every call — correctly — and, pointed at a motionless
bench, remarked on the same motionless desk every 15 s. Turning up the wording
changed nothing, because the wording was never the problem.

**The fix is not a better sentence. It is to decide on-device and omit the
function declaration.** `gemini_backend.c`'s `build_tools(bool allow_speak)`
drops the `speak` declaration entirely on a cycle where the robot should stay
quiet. An instruction is advisory; an absent tool is enforcement.

> The test that tells the two apart: *could a competent human answer this
> instruction given only this request?* If not, no phrasing will fix it — move
> the decision to where the state actually lives.

Corollaries that fell out of the same work:

- **Feed back what the model cannot remember.** The recall ring
  (`dialogue_style.h`) exists because "don't repeat yourself" is unactionable
  without a list of what was already said.
- **Then verify the answer anyway.** `dialogue_style_is_repetitive()` re-checks
  the returned line on-device. A request the model can quietly ignore is not a
  guarantee — and a *near*-repeat honours "don't repeat" while sounding
  identical.
- **Constrain the axis you actually care about.** The first anti-repetition
  mechanism compared only the first three *words*, so the model complied by
  reordering the same sentence forever. Word-set similarity
  (Sørensen–Dice, case- and punctuation-insensitive) closes that.
- **Gate the tool, not the sentence, when you want silence.** Screening the
  output still pays for the tokens that produced it; withholding the tool makes
  the quiet request the *cheaper* one, because the whole speech half of the
  prompt goes away with it.
- **Screen only generated output.** A console audition is meant to repeat on
  demand and a status report whose facts have not changed is supposed to read the
  same. Applying a novelty filter to either turns a working command into one that
  silently does nothing.

## 2. A fixed prompt buffer silently truncates the tail — reserve for it

Prompts are assembled with `snprintf` into a stack buffer on the calling task
(`GEMINI_SYSTEM_PROMPT_MAX`, on the planner's stack). Once variable-length
material goes in — a persona brief, a recalled-lines list — saturation stops
being hypothetical, and `snprintf` truncates the **end**. The end is where the
load-bearing constraint usually sits:

```c
"Respond ONLY with function calls — no prose, no markdown."
```

Lose that and the model starts answering in prose, with no error, no warning, and
no obvious cause — the failure surfaces in the *parser*, one layer away from the
buffer that caused it.

- **Assemble optional clauses before the closing constraint, and size them
  against the room actually left**, not against a comfortable-looking margin.
- **Measure the reserve with `sizeof` off the literals themselves** so editing
  the wording cannot put the arithmetic out of date:
  ```c
  static const char k_prompt_closing[] = "Respond ONLY with function calls …";
  #define GEMINI_PROMPT_TAIL_RESERVE (sizeof(k_prompt_closing) + …)
  ```
- **Grow `<TASK>_STACK_SIZE` in the same commit as the buffer.** The planner went
  8 K → 10 K when the prompt frame passed 4 KB. Doing it after a stack-overflow
  panic reports it is doing it twice.
- Prefer dropping a whole optional entry over truncating one: a half-quoted
  string in a prompt reads as a mangled instruction (and can split a UTF-8
  sequence on its way into a JSON body).

## 3. An on-device gate must measure something the hardware isn't already changing

The scene-change gate (`scene_change.c`) only works because of one decision:
block luma is stored as a **deviation from the frame mean**, never as an absolute
value. The sensor's AGC/AEC rewrites gain and exposure every single frame (see
`camera-sensor-identity.md`), so a completely static scene drifts in absolute
brightness continuously. An absolute comparison would have reported "the scene
changed!" on every passing cloud — reintroducing exactly the chattering the gate
was built to stop, and looking like a tuning problem rather than a design one.

The general form: **before trusting a threshold on a sensor-derived measurement,
ask what the sensor's own control loops do to that measurement when nothing in
the world changes.** Subtract that out, or the gate measures the driver instead
of the scene.

Two more gate-design points from the same module:

- **Pick the reference frame from the question you're asking.** The scene
  reference is the frame the robot last *spoke about*, not the previous frame.
  Per-frame comparison answers a different question and fails twice over: a slow
  drift never trips the threshold (the room can change completely in silence),
  and a just-changed scene reads as stale on the very next frame, before anything
  has been said about it.
- **Distinguish "cannot measure" from "measured zero".** An undecodable frame's
  fingerprint is all-zero, which is byte-identical to a flat grey frame. Without
  an explicit `valid` flag, a run of corrupt captures reads as a brand-new view
  and triggers speech about nothing. Same trap as
  `~/.claude/rules/diagnose-at-the-failure-point.md` #1: a zero that means
  "uninitialised" masquerading as a real reading.

## Where to put the knobs

None of these thresholds — speaking interval, per-window cap, repetition
percentage, scene-change distance — has a defensible compile-time default. Each
depends on the room, the lens, or on taste, so each is a **console knob**
(`voice quiet|budget|repeat|scene`) and the value that gates behaviour is
**logged every cycle** (`scene: <score>/<threshold>`) so it can be replaced with
a measured one. Guessing a constant, shipping it, and waiting for a reflash per
trial is the slow loop this repo keeps learning to avoid — same argument as
`cam gainceiling` in `camera-sensor-identity.md`, and the general form is
`agent-patterns-plugin:expose-tunable-knob`.

Deliberately **not** persisted: a boot should come up at the documented default,
not at whatever last night's experiment left behind.

## Verify

The behavioural claims here ("goes quiet when nothing happens", "speaks up when
something does") are cheap to pin in a host test and nearly impossible to
reproduce on a bench, where *nothing changed* is never quite true. Both modules
are pure C with injected state (clock, PRNG, luma plane) precisely so the
untestable-on-hardware cases are one `memset`:

```sh
just robocar-unified::test   # test_speech_budget, test_scene_change, test_dialogue_style
```

The load-bearing cases are the ones a bench cannot stage: an exact +80 brightness
shift with no motion (must score 0), and the uint32 millisecond wrap at day 49
(must not mute the robot forever).
