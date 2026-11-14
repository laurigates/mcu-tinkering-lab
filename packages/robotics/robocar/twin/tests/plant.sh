#!/bin/sh
# The plant exchange ("plant":"external", ADR-036 section 3) without a plant:
# the twin reads each plant_response from stdin when it asks, so the replies
# can be written ahead. 100 ms is four executor ticks (0, 33, 66, 99 ms).
set -eu
twin="$1"
config='{"t":"config","seed":1,"duration_ms":100,"plant":"external"}'
frame='{"t":"frame","at_ms":0,"scene":"desk"}'
run='{"t":"run"}'
reply() {
    printf '{"t":"plant_response","id":%s,"range_cm":{"front":%s,"rear":null},"floor":{"front":true,"rear":true}}\n' "$1" "$2"
}
fail() {
    echo "FAIL: $1" >&2
    exit 1
}

# One request per tick, in order, each answered. Replies 1 to 3 carry ranges
# the rangefinder cannot report; here that shows only that the run goes on.
# What they do to the wheels is checked below.
out=$({
    printf '%s\n' "$config" "$frame" "$run"
    reply 0 38
    reply 1 1
    reply 2 401
    reply 3 38.5
} | "$twin")
printf '%s\n' "$out"
[ "$(printf '%s\n' "$out" | grep -c '"t":"plant_request"')" = 4 ] || fail "want 4 plant_request lines"
for want in \
    '{"t":"plant_request","id":0,"at_ms":0,"dt_ms":33,"left":{"drive":"coast","duty":0},"right":{"drive":"coast","duty":0}}' \
    '{"t":"plant_request","id":3,"at_ms":99,"dt_ms":33,"left":{"drive":"coast","duty":0},"right":{"drive":"coast","duty":0}}' \
    '{"t":"done","at_ms":100}'; do
    printf '%s\n' "$out" | grep -qxF "$want" || fail "missing line: $want"
done

# What a plant range does to the wheels, over 400 ms (13 ticks, ids 0..12),
# with the same front value in every reply. The tick-0 trigger is latched
# before any reply and fails at its read (132 ms). The trigger at 165 ms ranges
# the reply's value, and the read at 297 ms is the first that can trip the
# front reflex (STOP_THRESHOLD_CM 15), which brakes with no goal set.
replies() { # $1 the front value as JSON, $2 the first id, $3 one past the last
    i=$2
    while [ "$i" -lt "$3" ]; do
        reply "$i" "$1"
        i=$((i + 1))
    done
}
config400='{"t":"config","seed":1,"duration_ms":400,"plant":"external"}'
ranged() { { printf '%s\n' "$config400" "$frame" "$run"; replies "$1" 0 13; } | "$twin"; }

# 5 cm is a reading: one wheels record, the brake at 297 ms, written before
# that tick's plant_request (id 9), which carries the brake to the plant.
brake='{"t":"wheels","at_ms":297,"left":{"drive":"brake","duty":0},"right":{"drive":"brake","duty":0}}'
request='{"t":"plant_request","id":9,"at_ms":297,"dt_ms":33,"left":{"drive":"brake","duty":0},"right":{"drive":"brake","duty":0}}'
out=$(ranged 5)
[ "$(printf '%s\n' "$out" | grep -c '"t":"wheels"')" = 1 ] || fail "front 5: want exactly one wheels record: $out"
printf '%s\n' "$out" | grep -qxF "$brake" || fail "front 5: no brake at 297 ms"
printf '%s\n' "$out" | grep -qxF "$request" || fail "front 5: request 9 does not carry the brake"
b=$(printf '%s\n' "$out" | grep -nxF "$brake" | cut -d: -f1)
r=$(printf '%s\n' "$out" | grep -nxF "$request" | cut -d: -f1)
[ "$b" -lt "$r" ] || fail "the wheels record (line $b) must come before that tick's plant_request (line $r)"

# A fraction, null and a value outside 2..400 are failed reads. A failed read
# counts as a clear path, so the wheels never change.
for v in 5.5 null 1 401; do
    out=$(ranged "$v")
    if printf '%s\n' "$out" | grep -q '"t":"wheels"'; then fail "front $v must be a failed read, but the wheels changed"; fi
    printf '%s\n' "$out" | grep -qxF '{"t":"done","at_ms":400}' || fail "front $v: no done record"
done

# A null range is a failed read to the planner too, not 400 cm. The planner
# wakes at 15 s on the 120 cm it has read since boot (400 cm) and plans on it.
# From then on the plant answers null, so the 30 s cycle has no range to
# compare: its verdict is "+", the ladder's next step. A null read as a valid
# 400 cm would be 280 cm off the 120 cm planned on, and the verdict "R".
# Ticks 0..454 run before the planner tick at 15 s and 455..909 before the one
# at 30 s; each planner tick asks the model once.
config30='{"t":"config","seed":1,"duration_ms":30100,"plant":"external"}'
link='{"t":"link","at_ms":0,"wifi":true,"key":true}'
plan() { # $1 the request id: a reply that calls stop
    printf '{"t":"model_response","id":%s,"status":200,"latency_ms":0,"body":"{\\"candidates\\":[{\\"content\\":{\\"parts\\":[{\\"functionCall\\":{\\"name\\":\\"stop\\",\\"args\\":{}}}]}}]}"}\n' "$1"
}
out=$({
    printf '%s\n' "$config30" "$link" "$frame" "$run"
    replies 120 0 455
    plan 1
    replies null 455 910
    plan 2
    replies null 910 913
} | "$twin")
printf '%s\n' "$out" | grep -qxF '{"t":"done","at_ms":30100}' || fail "null after 120: no done record"
printf '%s\n' "$out" | grep -q '"t":"cycle","at_ms":15000,"tick_ms":15000,"why":"R"' ||
    fail "null after 120: the 15 s cycle must wake on the range"
printf '%s\n' "$out" | grep -q '"t":"cycle","at_ms":30000,"tick_ms":30000,"why":"+"' ||
    fail "null after 120: the 30 s cycle must not see a range change: $(printf '%s\n' "$out" | grep '"t":"cycle"')"

# Each of these must end the run with the given status and a message on stderr.
# $1 status, $2 text the message contains, $3 name, stdin on fd 0.
refused() {
    status=0
    err=$("$twin" 2>&1 >/dev/null) || status=$?
    [ "$status" = "$1" ] || fail "$3: exit status $status, want $1 ($err)"
    case "$err" in
        *"$2"*) ;;
        *) fail "$3: stderr lacks '$2': $err" ;;
    esac
}
{ printf '%s\n' "$config" "$frame" "$run"; reply 1 38; } |
    refused 2 "plant_response id 1 does not answer request 0" "wrong id"
printf '%s\n' "$config" "$frame" "$run" '{"t":"model_response","id":0,"status":200,"latency_ms":0,"body":""}' |
    refused 2 "expected plant_response for request 0, got 'model_response'" "wrong record type"
{ printf '%s\n' "$config" "$frame" "$run"; reply 0 38; } |
    refused 3 "stdin closed while waiting for plant_response 1" "closed stdin"
printf '%s\n' "$config" "$frame" "$run" '{"t":"plant_response","id":0,"range_cm":{"front":38,"rear":null}}' |
    refused 2 "missing 'floor'" "no floor field"
printf '%s\n' "$config" "$frame" '{"t":"range","at_ms":0,"cm":50}' "$run" |
    refused 2 "'range' event at 0 ms" "range event with an external plant"
printf '%s\n' "$config" "$frame" '{"t":"range_fail","at_ms":5}' "$run" |
    refused 2 "'range_fail' event at 5 ms" "range_fail event with an external plant"
printf '%s\n' "$config" "$frame" '{"t":"floor","at_ms":7,"front":true,"rear":true}' "$run" |
    refused 2 "'floor' event at 7 ms" "floor event with an external plant"
{ printf '%s\n' "$config400" "$frame" "$run"; replies '"5"' 0 13; } |
    refused 2 "plant_response.range_cm.front must be a number or null" "a string range"
printf '%s\n' '{"t":"config","seed":1,"duration_ms":100,"plant":"physics"}' "$frame" "$run" |
    refused 2 'config.plant must be "external"' "unknown plant value"
echo "plant protocol OK"
