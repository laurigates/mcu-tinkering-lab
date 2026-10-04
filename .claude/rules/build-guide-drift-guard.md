# The Build-Guide Drift Guard: What May Feed It, and How to Verify a Change to It

`build-guide-check.yml` recompiles every `**/docs/build-guide.typ`, regenerates
its `docs/auto/pin_defs.typ`, and fails on any diff — so a pin renamed in the C
header cannot leave a *wrong* pin in a printed guide. The guard is path-filtered,
which is what makes it cheap and also what makes the two failure modes below
silent. Companion to `web-flasher.md` (the release/flasher path) and
`containerized-builds.md` (the ESP-IDF build path).

## 1. Only human-edited firmware constants may feed the generated file

`generate-pin-defs.py` reads the project through the hardware join
(`tools/hardware/`, issue #459), which takes its inputs from the project's
`hardware.toml`: `[source] header` (`main/pin_config.h`) plus `extra_headers` —
for robocar-unified the planner cadence headers `main/planner_task.h` and
`main/plan_activity.h` (issue #485). That list lives once, in the sidecar; the
justfile recipe and the guard both pass only the project directory. The
criterion is who writes the source, not whether it is hardware: a
constant only a human edits changes in a source commit that is on the guard's
trigger paths, so the guard runs. **Do not add an input that release automation
owns**, and **add every new input header to both trigger-path blocks** in
`build-guide-check.yml` — the trigger paths are the only thing that makes the
guard's verdict meaningful, and a file outside them changes the committed
artifact with no gate.

The concrete failure (issue #439, five hand-resyncs over three weeks):

| Step | What happened |
|---|---|
| 1 | The guide printed `VERSION`, generated from `version.txt` |
| 2 | release-please bumped `version.txt` — on **none** of the trigger paths |
| 3 | The guard never ran; the committed PDF kept printing the previous version |
| 4 | Regenerating it was a `docs:` commit, which release-please turned into the next release |
| 5 | → step 2 |

It never converged: each fix minted the release that invalidated it. The guide
self-healed only when an unrelated PR happened to touch `pin_config.h` and
dragged the regeneration along.

**Adding a `version.txt` glob to the trigger paths is the wrong fix.** It would make
the guard run on every release-please PR and *fail* it, because that PR's
committed PDF genuinely predates the version being introduced — turning a fully
automated release into a hand-held one.

The test before adding any input to a generated, committed artifact: **who writes
this file — a human changing hardware, or the release pipeline?** If the latter,
it does not belong in the join. Under ADR-021's framing (`pin_defs.typ` as a
board × header × parts join) the same answer falls out of asking whether the
field is hardware data at all.

The guide may still *name* `version.txt` as the source of truth in prose. It must
not interpolate its contents, and the shared template's `version:` parameter must
stay unset. `.claude/skills/build-guide/SKILL.md` carries this as a style rule so
a newly scaffolded guide cannot reintroduce the loop.

## 1a. A literal is invisible to the guard — interpolate every coordinate

The guard regenerates `pin_defs.typ`, recompiles, and fails on a byte diff. That
catches a stale page only where the page **interpolates** a generated binding. A
literal cannot go stale by construction: the PDF recompiles identically, the
guard reports `up to date`, and the page keeps printing a channel the firmware
stopped using.

Observed 2026-09: the build guide's PCA9685 channel map listed
`([8], [Motor R — IN1 (dir)])` through `([13], …)` as literals. The motor
channels were renumbered to follow the TB6612FNG's pin order and the table kept
printing the old assignment, green throughout — in the one document somebody
builds the robot from. Converting those rows to `#PCA_CH_MOTOR_R_PWM` and
friends produced a **byte-identical PDF**, which is the proof rather than a
coincidence: the guard could never have distinguished the two states.

`tools/check-typst-docs.py` now enforces it (pre-commit, so also CI): inside a
table whose first header cell is a hardware coordinate (`Ch`, `Channel`,
`GPIO`, `Pin`, `Pad`), a row's first cell may not be a bare integer. Ranges
(`14–15`), names (`D2`) and interpolations all pass. The report names every
binding sharing that value, because several do — channel 6 is both
`PCA_CH_SERVO_PAN` and `I2C_SCL_PIN`, and naming one would misdirect the fix.

## 1b. `git ls-files '**/docs/*.typ'` also matches `docs/auto/`

Git's **default** pathspec matching is fnmatch *without* `FNM_PATHNAME`, so a
bare `*` crosses `/`. The obvious widening of the guard's discovery therefore
also matches `<proj>/docs/auto/pin_defs.typ` — the generated include, which has
no PDF beside it and fails the "No committed PDF" branch on every run. The
pattern reads correctly, which is why review does not catch it.

Use the `:(glob)` magic, under which `*` stops at a separator and `**` spans
directories:

```sh
git ls-files ':(glob)**/docs/*.typ'      # documents only
git ls-files '**/docs/*.typ'             # also pin_defs.typ — the trap
```

`check-typst-docs.py` asserts the discovery set excludes `docs/auto/` and that
every discovered document has a committed PDF — and **control-tests itself** by
requiring the bare glob to still return a different set. If git ever changes
those semantics, the control fails loudly instead of the assertion quietly
becoming vacuous.

## 1c. An embedded schematic is an input too — re-render with `render-all`

`pin_config.h` and its sibling headers are not the only things that invalidate a
committed PDF. A document that embeds `docs/schematics/images/<name>.png`
compiles the image's bytes into the PDF, so re-rendering a circuit makes the
PDF stale with no `.typ` edit at all. The guard sees it — `docs/schematics/images/**`
is on its trigger paths — but `schematics-check.yml` passes on its own, so the
first sign used to be this guard failing in CI after the push (#588, fixed in
13c221a; issue #595).

**After changing a circuit, run `just schematics::render-all`, not `render`.** It
renders, then recompiles every document whose source references a schematic
image, through each project's own `build-guide` recipe so the pinned CLI and
flags match CI. Commit the images and the PDFs together. Plain `render` still
works and names any project it has just left stale.

The embedding documents are found by reading the `.typ` sources
(`docs/schematics/guide_deps.py`), not from a list, so a second guide that
embeds a schematic is covered without editing a recipe. It needs a
`build-guide` recipe in its project justfile; `render-all` fails rather than
skipping a project without one.

The PNG is the one artifact here that is not byte-reproducible across hosts —
cairo's encoder varies, which is why `schematics-check.yml` diffs only the SVG.
That does not weaken this guard: CI compiles the PDF from the *committed* PNG,
and with the pinned Typst CLI and flags the PDF is a deterministic function of
its inputs, so a PDF compiled from the committed PNG on any host matches. What
breaks the pair is committing one without the other.

## 1d. The board pinout images are generated, and the guard regenerates them

robocar-unified's build guide embeds one SVG per board
(`docs/auto/pinouts/<board-slug>.svg`), drawn by `tools/hardware/pinout.py` from
the board references hardware.toml names (`[source] board` and each part's
`board`, #629). Unlike the schematic PNG they are byte-reproducible text, so for
every document that references `auto/pinouts/` the guard regenerates them,
exactly as it regenerates `pin_defs.typ`, and fails if the result differs from
what is committed. A sibling document that embeds no pinout does not take them
into its drift set, so a stale image is reported against the guide alone.

Drift is read with `git status --porcelain`, not `git diff --quiet`: a board newly
given a `board` key produces an SVG nobody committed, and `git diff` does not see
untracked files. `pinout.py` also deletes an SVG no board produces any more.

`just robocar-unified::build-guide` runs `gen-pinouts` first, and
`just hardware::gen` regenerates them alongside everything else the join emits.
Commit the SVGs and the PDF together. Edit the board reference or hardware.toml,
never an SVG: the next regeneration overwrites a hand edit.

## 2. Verify a guard change by running the shipped script, with a negative control

Nothing else exercises this workflow — same gap as the flash recipes in
`containerized-builds.md` — so a change to its `run:` block is unverified until
you run it. **Extract the shipped text; never retype the logic into a harness**
(`~/.claude/rules/never-fabricate-test-identifiers.md`): a retyped copy that
misbehaves is indistinguishable from a real finding.

```sh
python3 - <<'PY' > /tmp/guard.sh
import re, sys, pathlib
wf = pathlib.Path('.github/workflows/build-guide-check.yml').read_text()
m = re.search(r'- name: [^\n]*check for drift\n        run: \|\n(.*?)(?=\n      - name:|\Z)', wf, re.S)
if not m:
    sys.exit("no '... check for drift' step with a run: block in build-guide-check.yml")
print('\n'.join(l[10:] if l.startswith(' '*10) else l for l in m.group(1).split('\n')))
PY
mise exec typst@0.15.0 -- bash /tmp/guard.sh
```

The snippet finds the step by the end of its name, `check for drift`, not the
whole name. The full name has changed once already ("every build guide" became
"every Typst document"), and the snippet then died on `m.group` before writing a
script (issue #637). `tools/check-typst-docs.py` runs this snippet exactly as
printed here and fails unless it emits a script that bash parses and that
contains `typst compile`. Its pre-commit hook fires on any commit touching the
workflow or this rule. It reads the pattern from this page, so a rename that
breaks the anchor fails that check, and the fix is the regex above.

Then run **both** controls. A green run alone proves nothing about a guard whose
condition you just edited:

| Control | Expect |
|---|---|
| Committed state, untouched | `up to date`, exit 0 |
| A pin changed in `pin_config.h` (e.g. `I2C_SDA_PIN` 5 → 9) | **exit 1**, stale-output error |

The negative control is the load-bearing one — it is the only evidence the guard
can still fail. Restore with `git checkout -- packages/<proj>/main/pin_config.h
packages/<proj>/docs/` afterwards.

**Run it from a clean tree, or its verdict is meaningless.** The guard recompiles
each PDF and then asks `git status --porcelain` whether the result matches what
is committed. So in a dirty working tree it reports the output as **stale whether
or not anything is wrong** — you just changed the source, so the regenerated
artifact legitimately differs from `HEAD`. Observed 2026-09: a correctly
regenerated PDF was reported stale, and the message ("Committed build-guide
output is stale…") reads exactly like a real finding.

The order is therefore: edit the `.typ`, run `just <proj>::build-guide`, **commit
the regenerated PDF and `pin_defs.typ`**, and only then run the guard. Both
controls above assume that commit has already happened.

**And read the guard's own exit code, not a pipe's.** `mise exec … -- bash
guard.sh | tail` makes `$?` the status of `tail`, so a guard that failed prints
`exit=0`. Redirect to a file and check the status, or test the command directly:

```sh
mise exec typst@0.15.0 -- bash /tmp/guard.sh > /tmp/guard.log 2>&1; echo "exit=$?"
```

The extracted script also references `TYPST_VERSION` in its failure message,
which the workflow sets as job-level `env:` rather than inside the `run:` block —
so export it before running, or a genuine failure dies on `unbound variable`
before it can tell you what was stale.

## 3. Do not grep a Typst PDF for text

Typst embeds subset fonts with custom glyph encodings, so searching the
decompressed streams for a string returns **zero hits even for text that is
plainly on the page**. A "the version is gone" verdict reached this way is
worthless, and it looks exactly like a real one.

Confirmed 2026-08-19: after decompressing every stream, `0.1.17` returned 0 hits
— and so did `XIAO ESP32-S3 Sense`, which is in the running header of every page.
The control is what exposed the method, not the result.

Check rendering instead, or let the guard's own byte-diff decide:

```sh
mise exec typst@0.15.0 -- typst compile --creation-timestamp 0 --ignore-system-fonts --root . --pages 1 --format png --ppi 110 packages/<proj>/docs/build-guide.typ /tmp/page1.png
```

Read the PNG. The title-page meta box and the running header are where the
template's optional fields render, so they are where an omitted parameter shows
up as a dangling label if a guard on `!= none` is missing.

## Related

- `web-flasher.md` — `flasher.json` scopes the release set, `project-matrix.json`
  scopes CI; the same "which file drives which pipeline" distinction
- `containerized-builds.md` § flash-recipe verification — the sibling case of a
  recipe CI never exercises
- `~/.claude/rules/never-fabricate-test-identifiers.md` — extract the shipped
  text; control-test every negative that gates an action
- ADR-021 (`docs/decisions/ADR-021-hardware-source-of-truth.md`) — the join that
  `pin_defs.typ` is an output of; since issue #459 the generator reads it through
  `tools/hardware/`
