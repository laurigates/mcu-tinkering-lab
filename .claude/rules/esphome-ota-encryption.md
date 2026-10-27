---
paths:
  - 'packages/**/*.yaml'
  - 'packages/**/secrets.yaml.example'
---
# ESPHome OTA: Encrypt With the API Key, Not a Password

Per the [ESPHome OTA docs](https://esphome.io/components/ota/esphome/#encryption),
the `esphome` OTA platform encrypts uploads with the same Noise protocol as the
native API. A new ESPHome config uses a bare `encryption:` block, which reuses
the API key, and has no `ota_password`:

```yaml
api:
  encryption:
    key: !secret api_encryption_key

ota:
  - platform: esphome
    encryption:
```

- **Leave out `key:` under `ota: encryption:`.** It defaults to the API key, and
  a different static key fails validation. Set it only on a device with no
  `api:` block (an MQTT-only device, for example).
- **`password:` and `encryption:` cannot be combined**, except for one
  migration install with `allow_plaintext_upload: true`.
- **Drop `ota_password` from `secrets.yaml.example`** once the config no longer
  references it.
- Check with `esphome config <file>.yaml`: the resolved `ota:` block shows
  `key: !secret 'api_encryption_key'`.

## Migrating a device that is already flashed

Changing the YAML alone can break the next wireless install. The CLI refuses to
send plaintext once `encryption:` is set, and a device running older firmware
cannot offer encryption. The plaintext fallback is removed in ESPHome 2027.3.0.

1. The device must run firmware built with ESPHome **2026.9.0 or newer**. Its
   log then shows `Encryption: offered, plaintext accepted`. If it doesn't,
   install once with the old `password:` config first.
2. Replace `password:` with `encryption:` and install over the air.
3. Confirm that the log shows `Encryption: required`.

To do it in a single install, add `encryption:` with
`allow_plaintext_upload: true` and keep `password:`. After that install, remove
both lines.

A device with no `api:` block needs a temporary `api:` with
`reboot_timeout: 0s` as a stepping stone. That also adds the API server and
Noise library to the image, so check the flash budget on 1 MB parts (ESP8285,
ESP-01).
