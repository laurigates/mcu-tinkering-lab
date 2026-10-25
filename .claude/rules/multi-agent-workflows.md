# Multi-Agent Workflows: Fair Inputs, Guarded Barriers, Clean Checkpoints

From the MG90D pan-tilt tournament (PR #861): 36 agents over two runs, one
session limit in the middle. Applies to any Workflow script that fans out
competing or parallel agents.

## Write the shared inputs before the fan-out

When agents compete or are compared, write the fixtures, gates and renderer
yourself first and point every agent at them. Five designers who each model
the servo get five slightly different servos, and the judges end up comparing
models. On the pan-tilt, `common/` (stand-ins, clash sweep, wall gate) came
first, and every entry was judged against the same geometry.

## Guard every barrier against an empty result

A failed agent resolves to `null`, and a session limit fails every agent still
running. Filter nulls before using a barrier's result, and handle the empty
case before indexing it: the first run died on `ranking[0][0]` when all three
judges hit the limit. `log()` what was dropped, so a partial run does not read
as complete.

## Resume, do not restart

After a session limit, fix the script and relaunch with `resumeFromRunId`:
completed agents replay from cache (seven did, including all five designs).
Read the journal before trusting cached results; an empty one replays as
empty.

## Commit only what finished agents wrote

While a workflow runs, its agents are still editing files. Committing to
quiet the stop hook then captures a design between edits, with a check report
that does not match its geometry. Commit at checkpoints: after a run, or
after a failed run, the files of the agents that completed. Before each
commit, re-run the gates on what is being committed and lint through
pre-commit (`python-lint.md`).

## Brief agents as peers, and verify their claims

Each agent knows only its prompt. Give it the shared paths, the interpreter to
use, and what "done" means (zero clashes, renders looked at). Then check the
results yourself: an agent reported a 0.01 mm wall as an artefact, and that
was right only after nine more seeds confirmed it. On the pan-tilt, inspectors
found 59 issues in designs whose designers reported success.
