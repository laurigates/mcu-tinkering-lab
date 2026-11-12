---
name: glitchtip-triage
description: "Read robocar resets, crash backtraces and planner notes from GlitchTip (tools/glitchtip) and take one from issue to fix. Use when the robot rebooted, crashed or misbehaved, after a field or bench session, when asked to check GlitchTip or review the planner's notes, or when bringing the GlitchTip stack up."
user-invocable: true
argument-hint: "[issue-id | project | query]"
allowed-tools: Bash(just:*), Bash(docker:*), Bash(git:*), Read, Grep, Glob, mcp__glitchtip__list_organizations, mcp__glitchtip__list_projects, mcp__glitchtip__list_issues, mcp__glitchtip__get_issue, mcp__glitchtip__get_latest_event, mcp__glitchtip__get_event
---

# GlitchTip Triage: From a Robot Event to a Source Line

The robot publishes Sentry-shaped events on MQTT `robocar/events`; the bridge in
`tools/glitchtip/` forwards them to a self-hosted GlitchTip, project `robocar`
(firmware: resets, crashes) or `robocar-ai` (planner notes). Read issues
through GlitchTip's MCP server when the `glitchtip` MCP tools are available,
otherwise with the recipes below; not the web UI.

Design and caveats: [ADR-033](../../../docs/decisions/ADR-033-robocar-error-reporting.md).
Stack, recipes, configuration: [`tools/glitchtip/README.md`](../../../tools/glitchtip/README.md).
What the firmware sends and when: [`packages/robotics/robocar/docs/firmware/error-reporting.md`](../../../packages/robotics/robocar/docs/firmware/error-reporting.md).

## 1. Is the stack up and receiving?

```sh
just glitchtip::issues          # fails with "no GLITCHTIP_API_TOKEN" or a connection error if not
```

If not: `just glitchtip::up`, `just glitchtip::bootstrap` (idempotent; writes
DSNs and the read-only API token to `tools/glitchtip/.env`), then start the
bridge, either `just glitchtip::bridge` on the host or the `bridge` compose
profile. Only one of the two: they share an MQTT client id and evict each other.

To prove the whole path, send a synthetic event from the robot and look for it:

```sh
just robocar::cmd "report test"   # needs MQTT full-access credentials
just glitchtip::issues robocar "report test"
```

No robot: `just glitchtip::send reset-brownout` (also `crash`, `planner-note`).

## 2. List, then read

**With the `glitchtip` MCP server** (registered in the repo `.mcp.json`; needs
`GLITCHTIP_API_TOKEN` in Claude Code's environment, printed by
`just glitchtip::mcp-env`). Arguments as GlitchTip 6.2.6 declares them:

| Tool | Arguments |
|---|---|
| `list_issues` | `organization_slug` (`robocar-lab`); optional `project_slug` (`robocar`, `robocar-ai`), `query` (`is:unresolved`), `sort`, `limit`, `start`, `end`, `environment` |
| `get_latest_event` | `issue_id`: the numeric id (`2`), not the short id (`ROBOCAR-2`) |
| `get_issue` | `issue_id` |
| `get_event` | `event_id`, `organization_slug` |
| `list_projects` | `organization_slug` |

`get_latest_event` returns the tags, the exception with its frames
(symbolicated frames carry the function, file and line) and the breadcrumbs.
The read-only token refuses `update_issue` (`Token lacks required scope`):
leave resolving issues to Lauri in the UI rather than asking for a write
token. If the `glitchtip` tools are missing, the token is not in the
environment or the stack is down; use the recipes.

**Without MCP**, the recipes print the same data as text:

```sh
just glitchtip::issues [project] [query]   # id, project, level, count, last seen, release, title
just glitchtip::event <issue-id>           # latest event: exception, frames, tags, extra, breadcrumbs
```

Add `--json` for the raw API response. The query is GlitchTip search syntax
(default `is:unresolved`).

## 3. Read each kind by its contract

| Kind | Project | Read |
|---|---|---|
| Reset (`Reset: BROWNOUT`) | `robocar` | `reset_reason`; breadcrumbs are the last ESP_LOG lines before the reset. Tag `breadcrumbs`: `survived`, `lost` (RTC memory cleared: power-on, maybe brownout), `empty` (the log hook was not running), `unavailable` (no PSRAM to copy into) |
| Crash (core dump) | `robocar` | Exception type is the Xtensa cause (`LoadProhibited`) or the panic's phrase (`abort() was called`); frames innermost first; `crash_task`; tag `symbolicated=true/false`; `backtrace=corrupted` means the walk stopped early, so trust the top frames only |
| Planner note | `robocar-ai` | The message is the model's own prose. Tags `trigger` (`reflex`, `blind`, `track_stuck`, `repeats`, `reflect`) and `kind` (`difficulty`, `behaviour`, `suggestion`); `[goal]` breadcrumbs are the last five goals; extras `reflex_trips`, `scene_score`, `still_score`, `plan_requests` |

`release` (`robocar@<version>`; `robocar-unified@<version>` from firmware up
to 0.2.23) and `build_sha` name the exact build.
Events for the same issue across releases show whether a fix took.

## 4. Unsymbolicated frames

`symbolicated=false` means the bridge had no ELF for that `build_sha`:

- **Local build:** `just glitchtip::keep-elf` right after building the
  firmware you flash. It files the ELF under the SHA embedded in it.
- **Released build:** the bridge fetches `robocar.elf` from the
  `robocar-v<version>` release and keeps it only if its embedded SHA
  matches. A `-dirty` build is never fetched; keep its ELF yourself.
- **Released build 0.2.18 to 0.2.23:** the bridge does not fetch these. Their
  ELF is `robocar-unified.elf` on the `robocar-unified-v<version>` release;
  download it and file it with `just glitchtip::keep-elf <path>`.

New events symbolicate once the ELF exists; old ones stay as they were.

## 5. From issue to fix

1. Open the frame's file and line, or grep the breadcrumb text to the
   `ESP_LOG` call that printed it.
2. Check out the build the event names (`git show <build_sha>`) when HEAD has
   moved; line numbers belong to that build.
3. Reproduce off the robot where the code allows: host suites
   (`just robocar::test`) for `*_core.c` logic, the behaviour twin
   (`just robocar-twin::test`) for planner decisions.
4. After the fix ships, the issue should stop collecting events for the new
   release. Hardware behaviour needs the bench; say so instead of claiming it.

## Traps

- **No wall clock on the robot.** An event's time is when the bridge received
  it, not when it happened; `extra.uptime_ms` is the robot's uptime when the
  event was queued.
- **An event waiting for MQTT is lost on a second reset.** A brownout loop that
  never reaches the broker reports nothing; the J15 UART capture is the record
  then.
- **A crash can appear twice, never zero times.** The dump is erased only after
  the broker acknowledges the event; a reboot or lost acknowledgement before
  that sends it again.
- **A robot updated over the air has no coredump partition** until it is
  flashed once over USB or from the web flasher; its panics arrive as plain
  `Reset: PANIC` events without frames.
- **A breadcrumb never shows the panic message itself:** the panic handler's
  output bypasses the log hook. The crash event's exception carries it instead.
