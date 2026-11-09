# robocar-unified: Error reporting to GlitchTip

The robot reports why it reset (and, with later additions, why it crashed
and what the planner found hard) to [GlitchTip](https://glitchtip.com), a
self-hosted server that speaks the Sentry protocol. Issues are grouped by
firmware release, so a fix can be checked against the build that had the
problem. Design and trade-offs:
[ADR-033](../../../../../docs/decisions/ADR-033-robocar-error-reporting.md).

## How it fits together

```
robot ── MQTT robocar/events (QoS 1) ──> broker ──> bridge ──> GlitchTip
                                                     └─ elf/<build_sha>.elf → source lines
```

The firmware holds no GlitchTip address and opens no extra TLS connection:
it publishes JSON on the MQTT broker it already uses, and the bridge in
[`tools/glitchtip/`](../../../../../tools/glitchtip/README.md) forwards it.
Moving GlitchTip to another host only changes the bridge's DSN.

## Setting it up

1. Start GlitchTip and the bridge: follow
   [`tools/glitchtip/README.md`](../../../../../tools/glitchtip/README.md)
   (`just glitchtip::up`, `just glitchtip::bootstrap`, then the bridge). Point
   `MQTT_URL` at the broker the robot uses (`MQTT_BROKER_URI` in
   `main/config.h` or `credentials.h`).
2. Check the whole path from the robot:
   `just robocar-unified::cmd "report test"` (needs MQTT full access; or type
   `report test` on the serial console). An `info` issue named
   `report test` appears in the `robocar` project, carrying this boot's last
   log lines as breadcrumbs.
3. `report` on the console shows the counters (queued, sent, dropped,
   pending) and whether the last boot's breadcrumbs survived its reset.

## The development loop

1. **Something goes wrong** on the robot: it reboots mid-drive, stops
   answering, or behaves oddly.
2. **Look in GlitchTip** (or ask Claude to; the `glitchtip-triage` skill
   reads issues through `just glitchtip::issues` / `glitchtip::event`). The
   issue's `release` and `build_sha` tag name the exact build; `reset_reason`
   says what kind of reset it was; the breadcrumbs are the last 32 log lines
   before it.
3. **Find the code.** Breadcrumb lines carry their log tag (`I (1234)
   planner: ...`), which names the source file; symbolicated frames name the
   function and line directly.
4. **Reproduce off the robot where possible**: a host unit test under
   `test/` for pure-C logic, or a behaviour-twin scenario
   (`packages/robocar/twin`, ADR-030) for planner decisions. Hardware timing
   still needs the bench.
5. **Fix, release, and watch** the issue: a new release with no new events
   for that fingerprint is the confirmation.

## What the robot reports

The robot publishes Sentry-shaped JSON on `robocar/events` at QoS 1, and the
host bridge in `tools/glitchtip/` forwards it to GlitchTip, grouped by
firmware release ([ADR-033](../../../../../docs/decisions/ADR-033-robocar-error-reporting.md)).
The JSON contract is in `main/event_report_core.h`.

- **Reset events.** On a boot whose reset was not deliberate, one event goes
  out once MQTT connects: `Reset: <reason>`, fingerprinted by the reason,
  tagged with `reset_reason`, `build_sha`, `version` and `device`. Power-on,
  `SW` (OTA, reboot), `EXT` (the reset button), `USB` (flashing, or opening
  the console), `JTAG` and `DEEPSLEEP` send nothing; every other reason,
  including `BROWNOUT` (issue #991), does.
- **Breadcrumbs.** Every ESP_LOG line that passes the UART0 mirror's hook is
  also kept in a 32-line ring in RTC memory, cut to 95 characters. A panic,
  watchdog or software reset leaves that memory alone, so the reset event
  carries the last lines before the reset. Power-on clears it. Whether a
  brownout keeps it is to be measured on the bench: `report` says whether the
  last boot's ring survived, and the event carries the tag `breadcrumbs`:
  `survived`, `lost` (RTC memory did not keep it), `empty` (kept, but the
  UART0 mirror's hook was not running, so nothing was written) or
  `unavailable` (kept, but there was no PSRAM to copy it into). The panic
  handler's own output does not pass through the hook, so the ring ends at the
  last ESP_LOG line before a crash, not at the panic message.
- **Held in RAM until MQTT connects.** A reset event that has not been
  published when the robot resets again is lost, and that boot's capture
  replaces the ring it carried. A brownout loop that never reaches MQTT
  reports nothing; the J15 capture is still the record for that case.
- **No wall clock.** The robot runs no SNTP, so `timestamp` is 0 and the
  bridge stamps the time it received the event; `extra.uptime_ms` is the
  robot's uptime when the event was queued.

## Don't

- Don't log from `event_report_breadcrumb()` or anything it calls, and don't let it wait. It runs inside the esp_log hook on whichever task logged, the 30 Hz executor included; a log line from it re-enters the hook. It takes a slot from an atomic counter and copies, with no lock
- Don't move `event_report_boot_capture()` after the first ESP_LOG line in `app_main`. The previous boot's breadcrumbs are still in the RTC ring until then, and the first line logged overwrites them
