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

# One request per tick, in order, each answered; ranges the rangefinder cannot
# report (null, 1, 401, 38.5) are accepted as failed reads.
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
printf '%s\n' '{"t":"config","seed":1,"duration_ms":100,"plant":"physics"}' "$frame" "$run" |
    refused 2 'config.plant must be "external"' "unknown plant value"
echo "plant protocol OK"
