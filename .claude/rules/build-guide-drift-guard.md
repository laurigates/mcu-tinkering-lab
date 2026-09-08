# The Build-Guide Drift Guard: What May Feed It, and How to Verify a Change to It

`build-guide-check.yml` recompiles every `**/docs/build-guide.typ`, regenerates
its `docs/auto/pin_defs.typ`, and fails on any diff — so a pin renamed in the C
header cannot leave a *wrong* pin in a printed guide. The guard is path-filtered,
which is what makes it cheap and also what makes the two failure modes below
silent. Companion to `web-flasher.md` (the release/flasher path) and
`containerized-builds.md` (the ESP-IDF build path).

## 1. Only hardware data may feed the generated file

`generate-pin-defs.py` takes `main/pin_config.h` and nothing else. Every `#let`
it emits derives from hardware. **Do not add an input that release automation
owns** — the guard's trigger paths are the only thing that makes its verdict
meaningful, and a file outside them changes the committed artifact with no gate.

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

## 2. Verify a guard change by running the shipped script, with a negative control

Nothing else exercises this workflow — same gap as the flash recipes in
`containerized-builds.md` — so a change to its `run:` block is unverified until
you run it. **Extract the shipped text; never retype the logic into a harness**
(`~/.claude/rules/never-fabricate-test-identifiers.md`): a retyped copy that
misbehaves is indistinguishable from a real finding.

```sh
python3 - <<'PY' > /tmp/guard.sh
import re, pathlib
wf = pathlib.Path('.github/workflows/build-guide-check.yml').read_text()
m = re.search(r'- name: Recompile every build guide and check for drift\n        run: \|\n(.*?)(?=\n      - name:|\Z)', wf, re.S)
print('\n'.join(l[10:] if l.startswith(' '*10) else l for l in m.group(1).split('\n')))
PY
mise exec typst@0.15.0 -- bash /tmp/guard.sh
```

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
each PDF and then asks `git diff --quiet` whether the result matches what is
committed. So in a dirty working tree it reports the output as **stale whether or
not anything is wrong** — you just changed the source, so the regenerated
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
  `pin_defs.typ` is an output of; issue #459 re-points the generator onto it
