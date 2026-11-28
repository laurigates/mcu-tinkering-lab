#!/usr/bin/env bash
# PreToolUse hook: before a PCB goes into a JLCPCB cart, tell Claude which
# export each cart line must match (zip stem, `r<N> <sha7> <pcb8>` silkscreen)
# and whether that export is dirty, stale or under a revision the order ledger
# says needs a bump. Advisory only: it answers with additionalContext and
# always exits 0. Logic and tests: tools/kicad/cartcheck.py.
payload="$(cat)"
# Every Bash and browser call comes through here: leave without Python unless
# the call mentions JLCPCB or JLCONE at all.
grep -qiE 'jlcpcb|jlcone|jlc\.com' <<<"$payload" || exit 0
cd "${CLAUDE_PROJECT_DIR:-$(dirname "$0")/../..}" 2>/dev/null || exit 0
PYTHONPATH=tools python3 -m kicad.cartcheck <<<"$payload" 2>/dev/null || true
exit 0
