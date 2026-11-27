---
name: pr-landing
description: "Land a PR here: merge-method rebase/squash rules, folding in-PR fixes, force-push tree checks, red checks inherited from main. Use when merging, rebasing, folding commits or force-pushing a PR."
user-invocable: false
---

# How a PR Lands: Rebase Eligibility and Which Components Release

The commit-type invariant (a commit under a released component's path carries
the type that component should see) stays always-loaded in
[`.claude/rules/pr-landing.md`](../../rules/pr-landing.md). This skill holds the
merge-method rules and the procedures for rewriting and checking a PR before it
lands.

On #1149 (2026-11-14) the PR changed the simulation, the behaviour twin and
two files under the firmware's path (`main/reactive_controller.{c,h}`, a
host-only test hook). Squashed as `feat(robocar-twin)`, it would have given
the `robocar` firmware a release entry for a host-only change and dropped the
`!` from the simulation's breaking change. Rebase-merged, only
`robocar-simulation` released.

## What turns the merge method to squash

The `merge-method` check (`merge-method.yml`; its rules are git-plugin's
`scripts/pr-merge-method.py` and `merge-strategies.md` in
`laurigates/claude-plugins`) labels a PR `merge:rebase` only when every rule
passes. Rules 4 and 8 applied on #1149; rule 5 is the other one a repair round
trips:

| Rule | Trigger | What to do |
|---|---|---|
| 4, merge commit | Any merge commit on the branch, including the one `auto-resolve-conflicts.yml` pushes when `main` moves | Bring `main` in by rebase, not by merge |
| 8, in-PR correction | A `fix`, `test`, `chore`, `refactor` or `style` commit whose files were all changed earlier in the PR, or a `fix` that shares its scope with earlier commits holding half its files | Fold it into the commit it corrects and reword that commit's message |
| 5, review churn | A subject matching "address review", "after review", "review findings", "wip", `fixup!` | Reword, or fold |

A `test` commit that adds new files is not a correction. A `docs` or `ci`
commit never is.

The auto-fix workflow pushes `fix(auto): resolve CI failure in ...` onto the
PR branch when a hook such as ruff format fails. It changes only files the PR
already touched, so it is a rule 8 correction and turns the label to
`merge:squash`. Fold it into the commit it fixes, then force-push with a
lease on the branch's current head (#1314). A `chore` is visible in this
repo's release-please config, so it does not count as the non-releasing last
commit a `merge:rebase` PR needs.

GitHub also cannot rebase-merge a branch whose commits conflict with `main`
when replayed one by one, even if a merge of `main` resolved the same
conflict: on #1149 the replay stopped at the commit that deleted a file `main`
had edited.

## A rebase merge can release a package its commits never touched

Release PR #1199 released esp32-cam-webserver, gamepad-synth and
switch-usb-proxy with one entry each: 4797d8c, `docs(tools)`, which changes
only `tools/esp32.just`. It was the top commit of #1162, rebase-merged, whose
other commits changed those packages' justfiles (#1202). The reported cause,
reproduced offline but not traced in release-please's source here:
release-please takes each commit's files from GitHub's GraphQL history and can
credit a rebase-merged PR's whole file list to its top commit when no other
commit of the PR falls in the same page of 10. Whether it happens depends on
the paging, so two runs on the same base gave the release PR 29 and 35 files.

The merge method follows the `merge-method` label (`.claude/rules/pr-landing.md`).
For a PR whose commits touch more than one release-please package:

- **`merge:squash`**: the squash commit's files are the PR's files, so the
  attribution is exact. It costs what the #1149 example above shows.
- **`merge:rebase`**: make the last commit one whose type releases nothing
  (`test`, `ci`, `build`, `style`, `refactor`; only `feat`, `fix`, `perf`,
  `docs` and `chore` are visible in `release-please-config.json`). Crediting
  it with the PR's files then adds no release. This follows from the reported
  cause and has not been seen on a release PR.
- **After the merge**, read the release PR's component list for a package
  none of the PR's releasing commits touched.

## Before the force-push

Rewriting a pushed branch needs Lauri's go-ahead. Then:

- **Compare trees, not logs.** Record `git rev-parse 'HEAD^{tree}'` of the
  head the gates passed on. After the rebase or the fold the hash must be the
  same; an unchanged tree is the one already verified and needs no second run.
  If `main` moved in between, the hash differs and the gates run again.
- **Fold by rebuilding, and move the branch last.** Cherry-pick the commits in
  order onto a detached `origin/main`, with `cherry-pick -n` and `commit
  --amend -F <message>` for each fold, and move the branch only when the tree
  matches.
- **Ask the classifier before pushing.** `decide()` in `pr-merge-method.py`
  can be imported and given the commits of `origin/main..HEAD` with their file
  lists from git. Classify the pushed history too, as the control: on #1149 it
  gave squash (rule 8) for the 15-commit history, matching CI, and rebase for
  the 12-commit one.
- **Pin the lease.** `git push --force-with-lease=<branch>:<sha>` with the
  remote head you read, so a bot's push in between fails the push.

## A red check that may be the base's

A check that fails on a PR for a file the PR never touched is usually red on
`main` as well. Show it before saying so: run the check's own command in a
detached worktree of `origin/main` and in one of the PR head (`git worktree
add --detach <dir> <ref>`), and `git diff --stat origin/main <head> -- <the
paths the check reads>`. Use worktrees, not `git archive` exports: a check
that lists files with git fails in an export for another reason, and both
sides then fail alike without having run. On #1149 `python3 -m kicad.stamp`
in an export printed "cannot list carrier projects; not inside a git work
tree?" on both sides; in worktrees it printed the same two `STAMP FAIL` lines
on both, with the balancebot stamp passing as the control.

Say it once on the PR, with the commands. After `main` moves, repeat the
`git diff --stat` for the new range before relying on the earlier comparison.
