# GlitchTip for robocar events

A self-hosted [GlitchTip](https://glitchtip.com) (Sentry-protocol error
tracker) plus a small bridge that forwards robot events from MQTT into it.
Resets, crash backtraces and planner notes land in GlitchTip, grouped by
firmware release. Design and wire format: [ADR-033](../../docs/decisions/ADR-033-robocar-error-reporting.md).

```
robot ──MQTT robocar/events (QoS 1)──▶ broker ──▶ bridge ──Sentry protocol──▶ GlitchTip
                                                    └─ elf/<build-sha>.elf (source lines)
```

The robot never talks to GlitchTip directly, so the firmware carries no
GlitchTip address or second TLS connection.

## Quick start

Needs Docker with compose v2, [just](https://just.systems) and
[uv](https://docs.astral.sh/uv/) (the last only to run the bridge on the host).

```sh
just glitchtip::up          # postgres + GlitchTip on http://localhost:8000; creates .env
just glitchtip::bootstrap   # admin, org, projects robocar + robocar-ai; DSNs into .env
just glitchtip::bridge      # run the bridge on the host (Ctrl-C to stop)
```

The admin login is in `tools/glitchtip/.env` (`GLITCHTIP_ADMIN_EMAIL`,
`GLITCHTIP_ADMIN_PASSWORD`). Registration is disabled.

### Broker

The bridge connects to `MQTT_URL` (default `mqtt://localhost:1883`). Point it
at the broker the robot already uses, or start the bundled Mosquitto with the
`broker` profile: set `COMPOSE_PROFILES=broker` in `.env` and run
`just glitchtip::up` again. The bundled broker allows anonymous access; keep
it on a trusted network.

### Running the bridge in a container

Set `COMPOSE_PROFILES=bridge` (or `broker,bridge`) in `.env` after
`bootstrap` and run `just glitchtip::up`. The container uses
`BRIDGE_MQTT_URL` and the `*_COMPOSE` DSNs, which point at the `web` service
instead of localhost. Run either the host bridge or the container, not both:
they share an MQTT client id and evict each other.

### Trying it without a robot

```sh
just glitchtip::send reset-brownout   # also: crash, planner-note
```

The `crash` fixture's addresses resolve against an ELF built from
`fixtures/tiny.c`; to see them symbolicated, compile it to
`elf/fixture.elf` (`clang --target=x86_64-linux-gnu -g -O0 -c`, or plain
`cc -g -O0 -c` on Linux).

## Development loop

1. **The robot misbehaves:** it reboots, crashes, or the planner hits
   something it notices. The firmware publishes one event on
   `robocar/events`; the bridge forwards it, and GlitchTip opens an issue
   (or counts another event on an existing one). What each kind carries is in
   [`error-reporting.md`](../../packages/robocar/unified/docs/firmware/error-reporting.md).
2. **Find it.** In the UI, or from the terminal:

   ```sh
   just glitchtip::issues                    # unresolved issues, newest first
   just glitchtip::issues robocar-ai         # one project: planner notes
   just glitchtip::issues robocar "BROWNOUT" # a GlitchTip search query
   just glitchtip::event 2                   # the latest event of issue #2
   ```

   ```
   #2 robocar    fatal   1x last 2026-11-09T08:01:25.121Z robocar-unified@0.0.0-fixture  LoadProhibited: PC 0x0000000a
   ```
   ```
   Exception: LoadProhibited: PC 0x0000000a
     frames (innermost first):
       0x0000000a add tools/glitchtip/fixtures/tiny.c:3
       0x0000002c entry tools/glitchtip/fixtures/tiny.c:8
   Tags: source=firmware, release=robocar-unified@0.0.0-fixture, build_sha=fixture, reset_reason=PANIC, symbolicated=true
   ```

   Add `--json` to either for the API's own response.
3. **Go to the source.** A crash names file and line once the build's ELF is
   known (below); a reset names its reason and the last log lines before it;
   a planner note names its trigger and the last goals. The `release` and
   `build_sha` tags say exactly which firmware it was.
4. **Fix and verify.** Reproduce in the host tests or the behaviour twin where
   the code allows, flash, and check that the issue stops collecting events
   for the new release. `report test` on the robot console (or
   `just robocar-unified::cmd "report test"` over MQTT) sends a synthetic
   event to confirm the whole path still works.

The recipes read with a read-only API token (scopes `org:read`,
`project:read`, `event:read`) that `bootstrap` creates and writes to `.env`
as `GLITCHTIP_API_TOKEN`, next to `GLITCHTIP_ORG_SLUG`. Re-running
`bootstrap` keeps the same token.

## Symbolication

Crash frames are resolved against `elf/<build_sha>.elf`, where `build_sha` is
the event's `tags.build_sha`, the exact string the firmware embeds
(`ROBOCAR_BUILD_SHA`). After building firmware you intend to run:

```sh
just glitchtip::keep-elf    # copies build/robocar-unified.elf to elf/<embedded sha>.elf
```

The SHA is read out of the ELF, not from git, so an older build is filed under
the commit it came from. Without a matching ELF the event still arrives,
tagged `symbolicated=false`, with raw addresses.

For a released build there is nothing to keep: when `elf/<build_sha>.elf` is
missing, the bridge downloads `robocar-unified.elf` from the
`robocar-unified-v<version>` GitHub release (attached by
`build-firmware.yml`) and keeps it only if the SHA embedded in it equals the
event's `build_sha`. A `-dirty` build is never fetched. Set
`ELF_RELEASE_URL` to another template (`{version}` is filled in) for a fork,
or to empty to turn fetching off.

## Claude Code (MCP)

GlitchTip has a built-in [MCP server](https://glitchtip.com/documentation/mcp)
on `/mcp` (Streamable HTTP), turned on in `compose.yml` with
`GLITCHTIP_ENABLE_MCP`. The repo's `.mcp.json` registers it as `glitchtip`, so
Claude Code in this repo gets its tools (`list_issues`, `get_issue`,
`get_latest_event`, `get_event`, `list_projects`, and the rest) once the
server is reachable and a token is in Claude Code's environment:

```sh
just glitchtip::mcp-env   # prints export GLITCHTIP_API_TOKEN=... and GLITCHTIP_MCP_URL=...
```

Add those two lines to your shell profile (or wherever you keep tokens) and
restart Claude Code; `/mcp` should list `glitchtip` as connected. The token is
the read-only one `bootstrap` creates, so an agent can read issues but
`update_issue` is refused (`Token lacks required scope`); resolve issues in
the UI. Without the token, or with the stack down, `/mcp` shows `glitchtip`
as failed to connect, which is harmless. To sign in with OAuth as your own
user (and your user's permissions, `update_issue` included) instead of the
token, add the server without the header at local scope:
`claude mcp add --transport http glitchtip http://localhost:8000/mcp`.

`just glitchtip::issues` / `::event` read the same data without MCP.

## Configuration (`.env`)

| Variable | Used by | Default |
|---|---|---|
| `COMPOSE_PROFILES` | compose | empty; `broker`, `bridge` |
| `GLITCHTIP_DOMAIN` | GlitchTip | `http://localhost:8000` |
| `GLITCHTIP_PORT` | compose, bootstrap | `8000` |
| `GLITCHTIP_SECRET_KEY` | GlitchTip | generated by `up` |
| `MQTT_URL` | host bridge | `mqtt://localhost:1883` (`mqtt://user:pass@host:port`) |
| `BRIDGE_MQTT_URL` | container bridge | `mqtt://mosquitto:1883` |
| `MQTT_TOPIC` | bridge | `robocar/events` |
| `GLITCHTIP_DSN_ROBOCAR`, `GLITCHTIP_DSN_ROBOCAR_AI` | host bridge | written by `bootstrap` |
| `*_COMPOSE` DSNs | container bridge | written by `bootstrap` |
| `ELF_DIR` | bridge | `tools/glitchtip/elf` |
| `GLITCHTIP_API_TOKEN`, `GLITCHTIP_ORG_SLUG` | `issues`, `event`, `mcp-env` recipes | written by `bootstrap` |
| `ELF_RELEASE_URL` | bridge (host run; the container keeps the default) | this repo's `robocar-unified-v{version}/robocar-unified.elf` release asset; empty turns fetching off |

`bootstrap` also reads `GLITCHTIP_ORG`, `GLITCHTIP_PROJECTS`,
`GLITCHTIP_ADMIN_EMAIL` and `GLITCHTIP_DSN_HOST` (see `bootstrap.sh`), so the
same stack can serve another project: pass other project names and route
them from the bridge's `PROJECT_BY_SOURCE`.

To move GlitchTip elsewhere, run `bootstrap` there (or create the projects in
its UI) and put the new DSNs in `.env`; the firmware does not change.

## Development

```sh
just glitchtip::test        # pytest; compiles a tiny ELF when the host can
just glitchtip::logs bridge
just glitchtip::down        # add -v to delete the data volumes
```

`bridge/src/robocar_glitchtip_bridge/event.py` and `symbolicate.py` are pure
(validation, routing, Sentry mapping, DWARF lookup); `main.py` holds the MQTT
and sentry-sdk I/O. `triage.py` is the `issues`/`event` CLI: its formatters
are tested against API responses recorded from GlitchTip 6.2.6
(`bridge/tests/fixtures/`); re-record them with `--json` after an upgrade.
