# Release-side requirements

The device-side code in this component is only half the story. Your CI
pipeline must publish artifacts in a specific shape for the device to
recognise and install them.

## Artifact contract

For each firmware release, the GitHub release must contain:

| Artifact | Purpose | Required by |
|---|---|---|
| `<your-name>.bin` | The app binary flashed to `ota_0` / `ota_1` | Both modes |
| `manifest.json` | Version/URL/SHA256 metadata for web flasher + optional verification | Web flasher |
| Release tag | Source of the semver string used for comparison | PULL mode |

### Asset naming

PULL mode picks the asset by **substring match** on its filename (via
esp_ghota's `filenamematch`). If you set
`cfg.firmware_filename_match = "robocar-camera"`, any asset whose name
contains `robocar-camera` matches. The monorepo convention is
`<project>.bin`.

### Tag format

This monorepo uses `release-please` with per-project tags:
`<project>@v<semver>` (e.g. `robocar-camera@v0.3.1`). esp_ghota extracts
the semver portion for comparison. Simple `v<semver>` tags also work.

## Reference CI workflow

The monorepo's [`_build-esp32-firmware.yml`][reusable] is a reusable
workflow that any project in this monorepo can call to:

1. Check out the release tag
2. Build firmware in the ESP-IDF Docker image
3. Compute SHA256, check the 1.8 MB OTA-partition limit
4. Rename the binary to `<project>.bin`
5. Generate `manifest.json` with URL, SHA256, size
6. Upload both to the GitHub release
7. (Optional) Publish the release tag to MQTT for push-notify

Key inputs:

| Input | What to set it to |
|---|---|
| `project` | Project directory name under `packages/esp32-projects/` |
| `target` | `esp32`, `esp32s3`, … |
| `binary_name` | Name of the `.bin` file inside `build/` |
| `mqtt_ota_notify` | `true` to publish the release tag to MQTT |
| `mqtt_notify_topic` | **New in this PR** — the topic to publish to, e.g. `myproject/ota/notify` |
| `release_tag` | The full tag being built |

A per-project caller looks like:

```yaml
# .github/workflows/build-myproject.yml
jobs:
  release-build:
    if: needs.check-tag.outputs.should_build == 'true'
    uses: ./.github/workflows/_build-esp32-firmware.yml
    with:
      project: myproject
      target: esp32
      binary_name: myproject.bin
      mqtt_ota_notify: true
      mqtt_notify_topic: myproject/ota/notify  # ← device subscribes to this
      release_tag: ${{ needs.check-tag.outputs.tag }}
    secrets: inherit
```

## Web Flasher integration (optional)

If you also want first-time USB flashing from a web page, see
[`docs/flasher/index.html`](../../../../docs/flasher/index.html) at the
monorepo root and [`.claude/rules/web-flasher.md`](../../../../.claude/rules/web-flasher.md)
for the partition-offset contract. The ESP Web Tools manifest is generated
dynamically by the release workflow from the same artifacts described above.

## MQTT broker considerations

- The reusable workflow uses [`mosquitto_pub`][mosq] with `-q 1` (at-least-once).
- Broker host/port come from `MQTT_BROKER_HOST` / `MQTT_BROKER_PORT`
  repository secrets.
- Publishing is gated on `vars.MQTT_ENABLED == 'true'` so forks without a
  broker don't fail the release build.
- The device-side rate limit is 60 seconds, so spamming the topic is
  harmless — use it freely for staged rollouts or canary releases.

[reusable]: ../../../../.github/workflows/_build-esp32-firmware.yml
[mosq]: https://mosquitto.org/man/mosquitto_pub-1.html
