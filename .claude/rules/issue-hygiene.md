---
paths:
  - '**/*'
---
# Issue Hygiene: File Fewer Issues Than You Close

On 2026-11-25 the tracker had 183 open issues, and the four weeks before had
opened 162 against 67 closed. Issues closed fast (median 2 days); the backlog
grew from what sessions filed: one bench-check issue per PR, plans split into
one issue per idea, and review findings filed one issue each. These rules cap
that inflow. Metadata and labels: `github-metadata-hygiene.md`.

## Before opening an issue

1. **Fold before filing.** A finding in a file the PR already touches, or a fix
   under about 30 lines, goes into that PR as its own commit, typed for the
   path it touches (`pr-landing.md`). No issue.
2. **At most one follow-up issue per PR.** Findings outside the PR go into one
   checklist issue, or onto the open rollup for that subsystem if one exists.
   Never one issue per finding. Close the rollup when its boxes are ticked.
3. **Bench checks go on the bench queue.** A change that needs a check on real
   hardware adds a checklist line to the pinned bench-queue issue for that
   robot (robocar, balancebot, or the one for everything else). Do not open a
   `[Test]` or "hardware-verify" issue. A bench session works the queue top
   down and ticks what it confirmed, with the commit it ran.
4. **Plans stay in the plan.** A PRD, ADR or epic lists future work as a
   checklist in its own file or epic issue. An item becomes an issue only when
   someone starts it, and the PR that does it closes it.

## Per session

5. **Net negative.** A session that opens N issues closes at least N, or says
   in its final reply which it opened and why it could not close as many.
6. **Soft cap of 20 open issues per `build:` or `area:` label.** Above it, new
   work for that label goes onto a rollup checklist instead of a new issue.
   Count with
   `gh api -X GET search/issues -f q='repo:laurigates/mcu-tinkering-lab is:issue is:open label:"build:robocar"' --jq .total_count`.

## Stale issues

7. **60 days without activity and no `needs:hardware`:** close as not planned
   with one line saying when to reopen. A parked idea goes into the project's
   PRD or README, not the tracker.

## Consolidating

When folding issues into a checklist, close each folded issue as not planned
with a comment linking the checklist issue, so its history stays findable.
