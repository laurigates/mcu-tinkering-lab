# Release-side requirements

The device-side code in this component is only half the story. The release
pipeline must publish artifacts in a specific shape for the device to
recognise and install them.

The monorepo previously planned a per-project reusable workflow
(`_build-esp32-firmware.yml`) for this. It was never actually wired up — its
only callers were dead `release-build` jobs whose tag-matching condition
never fired — and was deleted once that was confirmed (issue #409). The live
release path for **every** flasher-enabled project is the single
[`build-firmware.yml`][live] workflow, driven by each project's
`flasher.json` rather than per-project workflow inputs. See
[ADR-015](../../../../docs/decisions/ADR-015-ota-github-shared-component.md)'s
"Update" section for how the two diverged.

## Artifact contract

For each firmware release, `build-firmware.yml` attaches, per flasher-enabled
project:

| Artifact | Named by | Purpose | Required by |
|---|---|---|---|
| `<otaAssetName>` | `flasher.json` `otaAssetName` | The app binary flashed to `ota_0` / `ota_1` | Both OTA modes |
| `<project-id>-manifest.json` | project directory name | Version/URL/SHA256 metadata, also served from Pages | Web flasher |
| Release tag | release-please | Source of the semver string used for comparison | PULL mode |

`otaAssetName` is deliberately independent of `appBinaryName` (the name used
inside the web-flasher's manifest) — see issue #540. Set your device's
`cfg.triggered_asset_filename` (TRIGGERED mode) or
`cfg.firmware_filename_match` (PULL mode) to match your project's
`otaAssetName`, not its `appBinaryName`.

### Asset naming

PULL mode picks the asset with `fnmatch(cfg.firmware_filename_match,
asset_name, 0)` — a **full-string match**, not a substring match, despite
what this doc previously said. `cfg.firmware_filename_match = "my-project"`
will **not** match an asset named `my-project.bin`; you need either the exact
name (`"my-project.bin"`) or an explicit wildcard (`"my-project*"`).
`tools/check-ota-consumer-agreement.py` (wired into pre-commit) asserts this
agreement mechanically — a mismatch here fails locally before it fails on a
device.

### Tag format

This monorepo uses `release-please` with per-project tags in the form
`<component>-v<semver>` (e.g. `robocar-camera-v0.3.1` — see
`release-please-config.json`'s `component` field per package, and note it is
not always the same as the `packages/<domain>/<project>` directory name).
esp_ghota extracts the semver portion for comparison.

## MQTT notify topic

`build-firmware.yml` rebuilds and re-attaches every flasher-enabled project
on **every** release, regardless of which project's version actually bumped
— publishing to one shared topic used to wake every listening robot on every
release, with no way for a robot to tell whether the release was its own
(issue #409).

Each project instead declares its own `otaNotifyTopic` in `flasher.json`,
following the scheme `<component>/ota/notify` (the same `<component>` as the
release tag). The workflow parses the component out of the release tag it
was fired for, finds the `flasher.json` whose `otaNotifyTopic` matches, and
publishes only there — every other project's robot is left to its own poll
interval. Set your device's `cfg.mqtt_notify_topic` to your project's
`flasher.json` `otaNotifyTopic` exactly; `tools/check-ota-consumer-agreement.py`
checks this too.

The published payload is still the bare release tag, unchanged from before —
`ota_github_mqtt.c` in TRIGGERED mode uses the payload directly as the tag to
download, and PULL mode ignores the payload entirely (any message triggers a
check). Since the topic now already identifies the project, adding project id
or version to the payload would be redundant; the tag already carries the
version.

## Web Flasher integration (optional)

If you also want first-time USB flashing from a web page, see
[`docs/flasher/index.html`](../../../../docs/flasher/index.html) at the
monorepo root and [`.claude/rules/web-flasher.md`](../../../../.claude/rules/web-flasher.md)
for the partition-offset contract. The ESP Web Tools manifest is generated
dynamically by the release workflow from the same artifacts described above.

## MQTT broker considerations

- `build-firmware.yml` uses [`mosquitto_pub`][mosq] with `-q 1` (at-least-once).
- Broker host/port come from `MQTT_BROKER_HOST` / `MQTT_BROKER_PORT`
  repository secrets.
- Publishing is gated on `vars.MQTT_ENABLED == 'true'` so forks without a
  broker don't fail the release build, and on `github.event_name == 'release'`
  (a manual `workflow_dispatch` re-run never publishes).
- The device-side rate limit is 60 seconds, so a retried or re-dispatched
  publish to the same topic is harmless.

[live]: ../../../../.github/workflows/build-firmware.yml
[mosq]: https://mosquitto.org/man/mosquitto_pub-1.html
