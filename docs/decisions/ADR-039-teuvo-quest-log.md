# ADR-039: Teuvo's Quest Log: Self-Chosen Long-Horizon Tasks Kept on the Robot

**Status**: accepted (Lauri, 2026-11-27)
**Date**: 2026-11-27
**Source**: conversation 2026-11-27 (Lauri: Teuvo could decide to have a look
around, discover things, make notes on them and keep his own quest log)
**Confidence**: 4/10 (design only; nothing here has run. The curator's
cadence, the boredom delay and the per-quest ration are guesses until a bench
week sets them)
**Extends**: [ADR-016](ADR-016-hierarchical-ai-controller.md),
[ADR-022](ADR-022-planner-dormancy-and-spend-ceiling.md),
[ADR-029](ADR-029-idle-gaze-and-wander.md) (amends §4: wander starts by itself)
**Related**: [ADR-032](ADR-032-robocar-live-conversation.md) (Live
conversation, its `wait` tool), [ADR-033](ADR-033-robocar-error-reporting.md)
(planner notes), [ADR-030](ADR-030-robocar-behaviour-twin.md) (behaviour twin)

---

## Context

Teuvo has no time horizon longer than one planner request. Every
`generateContent` call is stateless (`.claude/rules/stateless-model-gating.md`):
the planner sees one frame, the rangefinders and the persona, and answers with
one goal. When nothing changes it goes dormant (ADR-022) and does nothing
until the room changes. The voice path remembers the last few turns of a
conversation (`voice_history.c`) and forgets them at reboot. Nothing remembers
what Teuvo has seen.

Lauri's proposal (2026-11-27): Teuvo decides by himself to look around,
discovers things, makes notes on them, and builds his own quest log. In the
same message: Teuvo should not fill silence or answer every utterance. The
Live session (ADR-032, PR #1318) already has a `wait` tool for that, and no
trigger word is wanted.

What the robot has to build on:

| Fact | Consequence for quests |
|---|---|
| No odometry, no map; `drive`/`rotate` are open-loop and capped at 3 s (`motion_bound_core.h`) | A place cannot be a coordinate. A discovery is located by what is near it and which dwell pose saw it |
| Idle gaze with four dwell poses and pose-keyed scene references (ADR-029 §1–2) | Looking around costs no request; a change at one pose is already an attention signal |
| Wander is not implemented and is off at boot by design (ADR-029 §4); cliff sensors exist (`cliff-sensors.md`) | Body motion for a quest depends on wander landing, and on Lauri switching it on |
| Planner fuse: 500 requests per boot, terminal (`plan_budget.h`) | Quests spend from the same fuse; a quest cannot be allowed to drain it |
| `note_gate`: the model gets a tool only when the device has a reason (ADR-033) | The same pattern decides when quest tools are offered |
| `spiffs` partition, 832 KB, unused (`partitions.csv`) | Room for a log on the robot without a partition change, so OTA-updated robots get it too |
| No SNTP; every timestamp is time since boot | "Yesterday" needs wall time; a note taken before sync has none |
| Planner prompt is a 3 kB stack buffer (`GEMINI_SYSTEM_PROMPT_MAX`) on an 11.5 kB task stack | The quest clause is a fixed, reserved length, not the whole log |

## Decision

A quest is device-held state that the models read and write through tools,
the way the recall ring feeds back what was said. The robot decides when a
model sees the log and when it may change it. A third, slow layer chooses the
quests; the planner pursues the active one; the executor is unchanged.

```
curator   (text only, flash-lite, minutes, on events)   chooses and closes quests
planner   (Robotics-ER, 15–300 s, on evidence)           pursues the active quest, logs discoveries
executor  (30 Hz)                                         gaze, track, bounded motion, reflexes
```

### 1. The log

A quest is `{id, kind, title, why, state, opened_at, closed_at, outcome}`.
A note is `{quest_id, at, pose, text}`. Text is model-written prose, capped
(title 60, why and outcome 120, note 160 bytes).

| Kind | Meaning | Example |
|---|---|---|
| `explore` | look around somewhere and note what is there | "What is behind the sofa?" |
| `find` | look for a named thing | "Find the red ball again" |
| `revisit` | check later on something noted | "Is the plant still drooping?" |
| `ask` | a question to put to a person, only when one talks to him | "Whose are the blue slippers?" |

States: `open`, `active` (at most one), `done`, `dropped`. At most 8 quests
are open or active; closed quests and their notes are kept up to a byte cap
and the oldest closed quest is pruned first.

**Storage**: LittleFS (`joltwallet/littlefs`) on the existing `spiffs`
partition, one append-only JSON-lines file, rewritten on prune. Pure C
(`quest_log_core.c`) owns the records, caps and pruning on an injected clock
and an injected store, host-tested and run by the twin. Notes survive reboots
and OTA.

**Wall time**: add SNTP (`esp_netif_sntp`). A note written before the clock
syncs is stored with its boot-relative time and flagged; prompts say "earlier"
for it, never a date.

**Mirror**: every change is published to `robocar/quests` on MQTT, retained,
as the current open quests plus the last 20 notes, so Lauri can read it from
the home server and the GlitchTip bridge host without the console. The robot's
copy is the source of truth; the mirror is read-only.

### 2. The curator chooses quests

A text-only `generateContent` call on a flash-lite model, beside the planner
on core 1. Its request holds the open quests, the last notes, the last few
voice-history lines (things people said), the time of day when known, and
whether wander is on. Its tools: `open_quest(kind, title, why)`,
`close_quest(id, outcome)`, `set_active(id)`, and `wait`. `wait` is the
expected answer most of the time, the same contract as the Live session's.

It runs on events, never on a timer alone:

| Trigger | Raised when |
|---|---|
| `bored` | the planner has been dormant for `QUEST_BORED_MS` (initially 10 min) |
| `discovery` | `QUEST_CURATE_NOTES` (initially 3) notes written since the last curation |
| `closed` | the planner closed the active quest |
| `talked` | a voice turn or Live session ended in which a person said something (voice history grew) |
| `boot` | the first curation after boot, once the clock has synced or 60 s have passed |

Rationed like `note_gate`: one curation per 5 minutes, 24 per boot, counted
per request, charged to `plan_budget` through `gemini_backend` (the choke
point). Text only, so a curation costs a fraction of a planner request.

Why a separate layer rather than quest tools on the planner: the planner's
prompt is a fixed 3 kB buffer that already holds the persona, range clause and
speech clauses, and choosing a goal over hours is a different question from
"what do I do about this frame". The curator sees the whole open log; the
planner sees one clause.

### 3. The planner pursues the active quest

When a quest is active, the planner's request carries a reserved clause of at
most `QUEST_CLAUSE_MAX` bytes (initially 384): the quest's kind, title and why,
and its last two notes. Two tools are offered only then:

- `log_discovery(text)`: what he found and what is near it. The firmware adds
  the time, the dwell pose and the quest id. Offered on a cycle that had view
  evidence (`why: V`) or a `track` goal on the previous cycle, so a static
  frame cannot be noted over and over. A note whose text is a near-repeat of
  one of the quest's notes (the Sørensen–Dice check `dialogue_style` uses) is
  dropped.
- `quest_done(outcome)`: closes the active quest. Offered once the quest has
  at least one note or has used half its ration.

Motion stays the existing tools: `track` (often at speed 0, to look), `rotate`,
`drive`, `stop`, and `set_idle` once ADR-029 §5 lands. No new goal kind, no
aiming primitive; `planner_task.c` stays the only goal source.

### 4. Boredom wakes the planner, bounded per quest

Dormancy stays the default (ADR-022). With a quest active, `plan_activity`
gains one more piece of evidence: `bored` from §2 has fired and the curator
kept or chose a quest. The planner wakes with a new `why:` letter, `Q`, and
runs a **quest session**:

- at most `QUEST_SESSION_REQUESTS` planner requests (initially 12, about three
  minutes at 15 s), then dormancy again;
- the idle gaze keeps running, so each request sees a different dwell pose and
  looking around costs nothing between requests;
- any ordinary evidence (a person, a sound, motion) ends the quest session and
  the planner carries on as now, the quest clause still present;
- a quest that has used `QUEST_TOTAL_REQUESTS` (initially 48) across sessions
  without closing is dropped by the firmware with outcome "gave up", and the
  next curation sees that.

A quest-driven robot therefore spends at most one quest session per
`QUEST_BORED_MS`: about 72 planner requests an hour at the initial values,
against 240 for the pre-ADR-022 loop. The 500-request fuse still binds.

### 5. Body motion

Head-only until wander exists. A quest can use the idle gaze, look-only
`track` and in-place `rotate` (which the rangefinders and the cliff reflex
already govern). `drive` during a quest session is allowed only while wander
is on. The quest clause tells the planner which of the two applies, and
`drive` is withheld from the tool list when it does not.

**Wander starts by itself** (Lauri, 2026-11-27). This amends ADR-029 §4,
where wander was off at every boot and on only by a console command. Here the
sensors decide, not a person:

- Wander is on whenever the cliff reflex is armed (`cliff_armed` in
  `reactive_telemetry_t`) and both floor sensors read floor. If either
  condition fails, wander is off and quests are head-only. That covers no
  MCP23017, `CLIFF_REFLEX_AT_BOOT` 0, `cliff off`, a failed read (fail
  closed) and a cliff trip. The cliff reflex is what makes driving on a table
  safe without a person deciding, so wander depends on it.
- After a cliff trip, wander stays off for `WANDER_CLIFF_HOLDOFF_MS`
  (initially 5 min), and it stays off for the rest of the boot after
  `WANDER_CLIFF_TRIPS_MAX` (initially 3) trips, so a robot on a table edge
  stops trying.
- `idle wander off` is the one way to stop it, and it is kept in NVS, so the
  choice survives reboots. `idle wander auto` restores the default.
- Wander's moves are still the bounded turns and short forward steps from
  ADR-029 §4, and they never reverse.

### 6. Speech: quests are quiet

Quest work does not open `speak`. A discovery is not evidence for speech on
its own; the existing gates (scene, audio, budget) decide as now. Teuvo talks
about his quests when someone talks to him:

- the voice turn's and the Live session's system instruction get a short quest
  summary (active quest, two latest discoveries, any `ask` quest), so "what
  have you been up to?" has an answer;
- an `ask` quest is put to a person only inside a conversation the person
  started, at most once per conversation, and the Live model keeps `wait` for
  everything not addressed to it. No trigger word, no interruption;
- the Live session gets `remember(text)`: a fact a person tells him ("the ball
  is Aino's") becomes a note on the matching quest or on a standing `talked`
  quest. It cannot move the body (#947).

### 7. Console and observability

`quest` lists open quests and the last notes; `quest show <id>`, `quest drop
<id>`, `quest clear` (confirmation required), `quest off|on` (no curation and
no quest sessions; off persists in NVS, since it is a decision about the room,
like wander). The planner log line gains `quest: <id>/<used>` and the `Q`
letter. Curations log their trigger, the tool chosen and the tokens.

## What this does not do

- No map and no coordinates. "Behind the sofa" is a description the model
  wrote, not a place the robot can drive back to. A `find` quest succeeds by
  seeing the thing again, not by navigating to it.
- No notes about who people are. A note may say "a person sat on the sofa", not
  a name or a face. Names come only from what someone tells him through
  `remember`, and `quest clear` deletes everything.
- No quest work while the planner fuse or the curator ration is spent, and no
  self-reset of either.

## Consequences

- Teuvo has something to do when the room is still, and something to say when
  asked, without talking more.
- Spend rises from zero at rest to at most one quest session per boredom
  window, all of it under the existing fuse. `quest off` restores today's
  behaviour exactly.
- A third model call (the curator) and a filesystem on the robot. LittleFS
  adds a managed component and a mount at boot; a failed mount disables quests
  and is named in the self-report, like any other optional peripheral.
- The planner prompt grows by the reserved quest clause and two declarations.
  Check `GEMINI_SYSTEM_PROMPT_MAX` and `PLANNER_TASK_STACK_SIZE` per the voice
  rules.
- Quest logic lives in pure C cores (`quest_log_core.c`, `quest_gate.c`) so
  the twin can run a day of boredom, sessions and rations in seconds.

## Implementation order

Each step is a PR, testable before the next.

1. `quest_log_core.c` with host tests (caps, pruning, near-repeat drop,
   restore from a truncated file), LittleFS mount, SNTP, the `quest` console
   commands with manual `quest open`. No model involvement.
2. The planner clause and `log_discovery`/`quest_done`, with quests opened by
   hand from the console. Bench: a hand-opened `find` quest closes when the
   thing is put in view.
3. The curator and its triggers, `bored` evidence and quest sessions in
   `plan_activity`/`planner_cycle_core.c`, twin scenarios for the rations.
4. The MQTT mirror.
5. Voice and Live: the summary in the system instructions, `ask` quests,
   `remember`.
6. Wander with automatic start (§5), then `drive` in quest sessions. This
   replaces ADR-029 step 6.

## Step 1 as built (2026-11-27)

Three changes from §1 and §7, made in the step-1 PR:

- **SPIFFS, not LittleFS.** SPIFFS is part of ESP-IDF, so the log needs no
  new managed component and no `dependencies.lock` change. The whole-file
  write through `log.new` and a rename, plus a parser that skips a torn last
  line, gives the power-loss behaviour LittleFS was chosen for. Revisit if the
  log moves to appends.
- **Tab-separated lines, not JSON lines.** Text is cleaned of control bytes on
  the way in, so a tab-separated record needs no escaping, and
  `quest_log_core.c` needs no cJSON. The host suite therefore always builds,
  even where cJSON is missing. Every line still starts with a format version.
- **`quest off|on` waits for step 3.** Until the curator and quest sessions
  exist, there is nothing for it to switch.

## Resolved questions (Lauri, 2026-11-27)

| Question | Ruling |
|---|---|
| Who chooses quests: a separate curator call or the planner alone? | A separate curator call (§2) |
| May Teuvo move his body during a self-chosen quest? | Head, look-only `track` and in-place `rotate` at any time; `drive` only while wander is on (§5) |
| What turns wander on? | Nothing by hand: it starts by itself while the cliff reflex is armed and reading floor; `idle wander off` (NVS) stops it (§5, amends ADR-029 §4) |

## Open questions

1. Curator model: flash-lite text-only is assumed; whether Robotics-ER with no
   image is cheaper or better at choosing is a probe item.
2. `QUEST_BORED_MS`, `QUEST_SESSION_REQUESTS`, `QUEST_TOTAL_REQUESTS`: set on
   the bench from a week of logs.
3. Whether to keep a 160×120 thumbnail per discovery (about 3 kB each, 832 KB
   partition) so a `revisit` can compare, and so the mirror can show it.
4. Whether quests should survive a firmware downgrade that predates the file
   format (a version field in each line is planned either way).
