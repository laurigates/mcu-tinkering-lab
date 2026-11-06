#!/bin/sh
# Smoke test: a two-event timeline through robocar-twin must end in a done
# record. No link event, so the woken request takes the no-link path
# (ESP_ERR_INVALID_STATE, no model_request) and needs no model reply.
set -eu
twin="$1"
out=$(printf '%s\n' \
    '{"t":"config","seed":1,"duration_ms":20000,"planner_period_ms":15000}' \
    '{"t":"frame","at_ms":0,"scene":"desk","pose":"C"}' \
    '{"t":"console","at_ms":5000,"line":"status"}' \
    '{"t":"run"}' | "$twin")
printf '%s\n' "$out"
printf '%s\n' "$out" | grep -q '^{"t":"done","at_ms":20000}$' || { echo "FAIL: no done record" >&2; exit 1; }
printf '%s\n' "$out" | grep -q '"t":"force_stop","at_ms":15000,"reason":"no_link"' || { echo "FAIL: no no_link stop at 15 s" >&2; exit 1; }
if printf '%s\n' "$out" | grep -q '"t":"model_request"'; then echo "FAIL: request without a link" >&2; exit 1; fi
echo "smoke OK"
