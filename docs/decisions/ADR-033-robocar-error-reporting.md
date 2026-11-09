# ADR-033: Robocar Error Reporting through GlitchTip

**Status**: accepted
**Date**: 2026-11-09
**Confidence**: 7/10 (the stack, bridge and symbolication run end to end
against fixture events; nothing here has run on the robot yet)
**Related**: [#991](https://github.com/laurigates/mcu-tinkering-lab/issues/991)
(brownouts on motor start), [ADR-030](ADR-030-robocar-behaviour-twin.md)
(behaviour twin)

---

## Context

Finding out why the robocar misbehaved means watching the USB console or the
J15 log mirror live, or reading MQTT logs by hand. A brownout (#991), a panic
or a watchdog reset is visible only on whichever console happened to be
attached. Three kinds of signal should land somewhere searchable, grouped by
firmware release:

1. Why the robot reset.
2. Crash backtraces, resolved to source lines.
3. Notes the planner writes about its own behaviour and the difficulties it
   ran into, as input to firmware development.

GlitchTip is a self-hosted server that speaks the Sentry protocol. The stack
lives in this repo as a docker compose setup, so other projects and community
members can start it without any particular cluster.

Facts the design rests on:

- MQTT is in daily use: the broker is `MQTT_BROKER_URI`
  (`packages/robocar/unified/main/config.h`), topics are `robocar/*`, and
  `mqtt_logger_publish()` already publishes from the robot.
- `reset_reason_name()` exists in `main.c`.
- The planner's side-channel tools (`speak`, `sound`) follow one
  declare-in-`gemini_backend.c`, parse-in-`gemini_parse.c` pattern.
- The app image is 1.50 MB in a 3.5 MB OTA slot; the 960 KB `spiffs`
  partition is never mounted; there is no coredump partition.
- Releases do not attach the `.elf`.

## Decision

The robot publishes JSON shaped like a Sentry event on `robocar/events` at
QoS 1. A bridge on the host forwards each event to GlitchTip with the official
`sentry-sdk`, and adds what the robot cannot: source lines for crash
addresses, routing to the right project, and receive time when the robot has
no clock. The firmware opens no new TLS connection and knows no GlitchTip
address, so moving GlitchTip to another host only changes the bridge's DSNs.

### Wire format (`robocar/events`, QoS 1)

```json
{"event_id": "<32 lowercase hex>", "timestamp": 0, "level": "error",
 "source": "firmware", "release": "robocar-unified@<version>",
 "message": "reset: BROWNOUT", "fingerprint": ["reset", "BROWNOUT"],
 "tags": {"reset_reason": "BROWNOUT", "build_sha": "<ROBOCAR_BUILD_SHA>"},
 "extra": {"uptime_ms": 4210},
 "breadcrumbs": [{"message": "<log line>", "category": "log"}],
 "exception": {"type": "LoadProhibited", "value": "PC 0x42001234",
               "frames": [{"instruction_addr": "0x42001234"}]}}
```

- `source` is `firmware` or `planner`; the bridge routes them to the
  GlitchTip projects `robocar` and `robocar-ai`.
- `timestamp` 0 means the robot had no SNTP time; the bridge uses its receive
  time and sets `extra.robot_timestamp_missing`.
- `tags.build_sha` is the exact `ROBOCAR_BUILD_SHA` string (40 hex, maybe
  `-dirty`); the bridge symbolicates against `elf/<build_sha>.elf`.
- `exception` is optional. Frames go innermost first, as ESP-IDF's backtrace
  prints them; the bridge reverses them to Sentry's outermost-first order.
  Caller addresses already point at the call instruction (ESP-IDF subtracts 3
  from each return address), so they are resolved as-is.

### Four PRs, one theme each

1. **Stack and bridge** (`tools/glitchtip/`). Compose file based on upstream's
   sample, trimmed to Postgres plus one `SERVER_ROLE=all_in_one` GlitchTip
   container with Valkey disabled. Mosquitto under the `broker` profile for
   anyone without a broker; the bridge under the `bridge` profile.
   `bootstrap.sh` creates the admin, org and projects through `manage.py
   shell` (the REST bootstrap endpoints change between versions) and writes
   the DSNs to a gitignored `.env`. The bridge (uv project, `paho-mqtt`,
   `sentry-sdk`, `pyelftools`) keeps validation, routing and symbolication as
   pure functions with pytest tests. pyelftools reads DWARF in pure Python,
   so no Xtensa toolchain is needed.
2. **Reset events and breadcrumbs** (firmware). An ESP-IDF-free
   `event_report_core` builds the JSON from a struct, with a host test; a
   Core 1 queue publishes through `mqtt_logger_publish()` once MQTT connects.
   One event per boot whose reset reason is not POWERON or SW. A ring of the
   last ~32 log lines, filled from the `log_mirror` hook without logging or
   blocking, lives in `RTC_NOINIT` memory so the lines before a panic or
   watchdog reset reach the next boot's event. A `report test` console
   command sends a synthetic event.
3. **Core dumps** (one USB reflash per robot, for the partition table).
   `spiffs` shrinks from `0xF0000` to `0xD0000` and a 128 KB `coredump`
   partition is added at `0x7E0000`; the OTA slot offsets stay. At boot,
   `esp_core_dump_get_summary()` becomes an exception event, and the dump is
   erased only after the QoS 1 publish is acknowledged. `build-firmware.yml`
   attaches `robocar-unified.elf` to each release, and the bridge fetches it
   by version when `elf/<sha>.elf` is missing locally (not in PR 1).
4. **Planner notes**. A side-channel tool `note(kind, text)` with `kind` one
   of `difficulty`, `behaviour`, `suggestion` and at most 280 characters,
   declared beside `speak`. It is offered only after a deterministic trigger
   (reflex brake, a `track` goal that does not converge, BLIND, repeated
   repetition rejections, a periodic reflection), decided in
   `planner_cycle_core.c` so the behaviour twin runs it. The model writes the
   prose; the firmware attaches `trigger`, `kind`, the fingerprint
   `trigger+kind`, recent goals and frame stats. At most 1 note per 5 minutes
   and 20 per day, with its own fuse that resets on `plan resume` or reboot.

## Consequences

- One more host service to run. Without the bridge the robot's events wait in
  the broker: the bridge's MQTT session is persistent (fixed client id,
  `clean_session=False`), so QoS 1 events published while it is down are
  delivered when it reconnects, for as long as the broker keeps the session.
  Run one bridge per broker: two with the same client id evict each other.
- A local build can be symbolicated only if its ELF was kept
  (`just glitchtip::keep-elf`). An unmatched crash still arrives, tagged
  `symbolicated=false`, with raw addresses.
- `CONFIG_ESP_TASK_WDT_PANIC` stays off: it changes behaviour (a stuck task
  would reboot the robot). It is a later option once dumps are trusted.

### Caveats

- A core dump captures panics, asserts, interrupt-watchdog resets and stack
  overflows, but **not brownouts or power loss**. #991 gets reset-reason
  counts and breadcrumbs, not dumps.
- Writing the dump adds time between the panic and the reboot, and the
  PCA9685 holds its last PWM through an ESP32 reset, so the robot can keep
  driving until boot re-initialises the motors. Measure that gap on the bench
  before relying on dumps while driving.
- The dump includes task stacks in PSRAM only if configured to capture them.
  Check which task stacks are external before trusting the backtraces.
- Whether the `RTC_NOINIT` breadcrumb ring survives a brownout reset is to be
  measured; it does not survive power-on.

## Alternatives considered

- **Sentry SDK or HTTP from the firmware.** A second TLS connection on a
  device whose internal RAM already ran out under overlapping handshakes
  (`CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC`), and a GlitchTip address baked into
  firmware. Rejected.
- **An existing MQTT-to-Sentry bridge.** None found.
- **Depending on an existing cluster deployment of GlitchTip.** Ties the stack
  to one person's setup; the compose file runs anywhere with Docker.
