---
reviewed: 2026-11-14
---

# How a PR Lands: Rebase Eligibility and Which Components Release

release-please reads the commits that reach `main`, each by its type and by
the paths it touches (`release-please-config.json`). A rebase merge keeps a
PR's commits; a squash replaces them with one commit under the PR title that
touches every path the PR touched. For a PR inside one component that makes no
difference. For one that spans components it decides what gets released.

On #1149 (2026-11-14) the PR changed the simulation, the behaviour twin and
two files under the firmware's path (`main/reactive_controller.{c,h}`, a
host-only test hook). Squashed as `feat(robocar-twin)`, it would have given
the `robocar` firmware a release entry for a host-only change and dropped the
`!` from the simulation's breaking change. Rebase-merged, only
`robocar-simulation` released.

## Write the commits for the paths they touch

- A commit under a released component's path carries the type that component
  should see. A host-only hook or test under the firmware's path is
  `test(robocar)`, which releases nothing.
- Keep a breaking change in one commit, with `!` and a `BREAKING CHANGE:`
  footer, touching only the component it breaks.
- List which commits touch which component before asking for the merge:
  `git show --format= --name-only <commit>` against the component's path and
  its `exclude-paths`.

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

GitHub also cannot rebase-merge a branch whose commits conflict with `main`
when replayed one by one, even if a merge of `main` resolved the same
conflict: on #1149 the replay stopped at the commit that deleted a file `main`
had edited.

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
