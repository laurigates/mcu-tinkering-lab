# robocar: Activity indicators

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

## Activity indicators (`activity_trace.c`)

Between two planner lines fifteen seconds apart the robot captures a frame, opens
TLS, uploads a JPEG, waits on a model and possibly streams back audio. A capture
that never returns and a request hung for its full timeout look identical from
the console — and off the tether there is no console at all. So both choke points
are instrumented: **every** picture comes from `camera_capture()` and **every**
endpoint call from `gemini_http_post()`, which is why the hooks live there rather
than at the call sites (a fifth caller then gets traced for free instead of
silently going dark).

The left LED is the camera, the right is the network:

| LED | Colour | Meaning |
| --- | --- | --- |
| left | white | frame captured (brief pulse) |
| left | red | last capture FAILED — **holds** |
| right | blue | request in flight — held for as long as it is |
| right | green | last request returned 200 (brief pulse) |
| right | yellow | last request was 429 rate-limited — **holds** |
| right | red | last request failed — **holds** |

**A held blue is the signal worth having**: a request that has not come back,
otherwise invisible until its timeout expires. Failure colours hold rather than
pulse so a fault that happened while nobody was watching is still on the robot.
`trace` totals it all since boot, with mean/max latency per endpoint and — when
something is wedged — how long the outstanding request has been outstanding.

`trace` also prints the **I2C bus** line, counted in `i2c_bus_select_channel()`
because that is the single gate every downstream transaction passes through —
PCA9685 writes, the expander, and anything added later. Instrumenting the
callers instead would let a new writer go uncounted, which is exactly the case
worth catching: a peripheral written on a timer is invisible from the console
and surfaces as a symptom somewhere else on the board. One "op" is one bus
acquisition (a mux channel-select plus the caller's device traffic), so it is a
lower bound on wire transactions, never an overcount.

An idle robot should read close to zero. A steady rate with nothing moving means
something is writing the bus on a timer. The `now` field is **zeroed once its
window goes stale** rather than reporting the last rate it saw: a remembered
rate on a silent bus is the one reading this counter exists to rule out.

**A colour is written only when it changes, and re-asserted every
`LED_REFRESH_INTERVAL_MS` even when it does not.** The suppression keeps a 25 Hz
task off the shared I2C bus; the re-assertion exists because the shadow it keeps
is a claim about a chip that cannot be read back. A PCA9685 that browned out or
was re-seated holds its power-on defaults — all outputs off — while the shadow
still says the colour is lit, and the states worth seeing are exactly the ones
that never change on their own: a held red for a failed capture, a held blue for
a request that never returned, a held yellow for a 429. Without the expiry a
stale indicator reads as "nothing happening", which is the opposite of what it
means. A failed write invalidates the shadow rather than being remembered as
displayed, and `trace led off` still means off — the refresh does not resume
writing. Pinned by `test_activity_trace.c`, including the uint32 millisecond
wrap at day 49.

Indicators may never cost the path they measure: the hot paths only stamp a few
words, all PCA9685 traffic happens on the indicator task (priority 2, below
everything it observes), LEDs are written only on a colour change, and the
counter mutex is taken with a 20 ms timeout and the update **dropped** on
contention. A lost count is a fair price; a stalled planner is not.
