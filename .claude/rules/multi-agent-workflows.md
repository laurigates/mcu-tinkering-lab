---
reviewed: 2026-11-13
---

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

## Give each agent its own scratch directory

Workflow agents share the session scratchpad, and agents in separate worktrees
still see it. Fixed names collide there: on the GY-521 orientation run
(#1015-#1022, 2026-11-09) parallel agents overwrote each other's
`commitmsg.txt`, `mutate.sh` and `merge-msg.txt` three times. Each agent
caught it only by re-reading its commit afterwards. Tell every agent to write
its scripts, commit messages and PR bodies under `tmp/` in its own worktree
(git-ignored), or under a scratchpad subdirectory named for its unit.

## Brief agents as peers, and verify their claims

Each agent knows only its prompt. Give it the shared paths, the interpreter to
use, and what "done" means (zero clashes, renders looked at). Then check the
results yourself: an agent reported a 0.01 mm wall as an artefact, and that
was right only after nine more seeds confirmed it. On the pan-tilt, inspectors
found 59 issues in designs whose designers reported success.

## Trial-merge every branch before landing any

Units verified one at a time can still break each other, and each PR's own
green CI cannot show it. Before the first merge, merge all the run's branches
onto `origin/main` in a throwaway worktree and run the shared suites. On the
core sweep (#1095-#1101, 2026-11-11) four of seven branches conflicted in
`.pre-commit-config.yaml` or `CLAUDE.md`, and two broke in ways no conflict
marker showed: #1099's new check called a helper #1100 had refactored away
(13 test errors), and #1096's ESP-IDF bump tripped a version tripwire #1100
had just added.

Land branches that conflict with each other as a stack: merge `origin/main`
into the first, the first into the second, and so on, pushing each. Each PR's
CI then runs on the cumulative content. After a lower PR is squash-merged the
next one reads as conflicting, because its history holds the same change as
separate commits. Re-sync it with `git merge -X ours origin/main` and compare
`git rev-parse 'HEAD^{tree}'` before and after: an unchanged tree is the one
CI already passed, so it can merge without another round.

## Agents share one GitHub rate limit

Every agent in a run draws on the same 5,000 REST calls an hour. The first
core wave (26 agents, 2026-11-10) used it up. `gh run list`, `gh run view`
and `gh api` then fail, while `gh pr` and `gh issue` (GraphQL) keep working.
The merge guard hook fails closed on the same limit and reports the PR as
"written by someone else"; read the 403 in its output before treating that as
an authorship finding, and retry after `gh api rate_limit` shows the reset.
Tell agents to use git for anything git can answer and not to poll `gh`.

## Watch for the bots on an agent's PR

A red check on a run's PR starts `auto-fix.yml`, which can push a commit to
the branch while the orchestrator is still deciding what to do (#1073: a
Linux-only `bash -c` argument-length fix, correct and kept). Run
`git log <head_sha>..origin/<branch>` before merging or re-syncing a branch,
so a commit nobody in the session wrote is read first. `auto-resolve-conflicts.yml`
runs on every push to `main` and can fail on a branch that is being merged at
that moment; check that the next run passes before reading it as a defect.

## Have agents report follow-ups, then verify them once and file

Tell implementers and verifiers to return out-of-scope defects in their
result, not to file them. An issue sweep whose agents file issues refills the
queue it is draining, and nobody has checked what they filed. On the
2026-11-11 run (#1115-#1121, 18 agents) the seven units returned about forty
follow-ups. After the PRs merged, one pass over `main` re-checked each claim
and filed nine issues (#1123-#1131). The rest were too small or already moot.

Reproduce the headline claim of each issue before filing, with a control:

- #1124: a cut line holding `inf` never returned from `validate_config`
  within 60 s, while the zero-length control returned its error.
- #1127: a mutant that unlocked before the flag update still passed the host
  suite three times out of three, after the unmutated control passed.

Mark in the issue body what was reproduced and what is only reported.

## Rebuild the PR body after a repair

An implementer drafts the PR body before its verifier runs. A repair commit
then changes the test counts and adds a fix the body does not mention: on
#1119 the body still said 802 tests and "8 failures, 3 errors" after the
repair made it 803 and 4, and on #1121 it said 8 tests where the suite had
18. The verifier's verdict lists these. Read each verdict's defect list for
"PR body" entries and correct the body before opening the PR.

## In a cloud session, tell agents how to read issues

A cloud session has no `gh` CLI for issue or PR reads. An agent told to run
`gh issue view` there fails or guesses. Say in the brief to read issues with
the GitHub MCP tool `mcp__github__issue_read` (methods `get` and
`get_comments`), and keep pushes and PR creation in the main session.
