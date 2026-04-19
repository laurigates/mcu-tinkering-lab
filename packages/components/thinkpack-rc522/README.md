# thinkpack-rc522

MFRC522 NFC reader driver (SPI) for ThinkPack firmware. Pure ESP-IDF
component: no project-specific includes, safe to consume from any
ThinkPack box.

## What it does

- Initialises the RC522 over SPI (bus + device + hardware reset).
- Polls for ISO14443A tags and reads either a 4-byte or 7-byte UID.
- Handles MIFARE cascaded-UID anticollision and the select handshake.

## Public API

```c
#include "thinkpack_rc522.h"

esp_err_t thinkpack_rc522_init(void);
bool      thinkpack_rc522_poll_tag(uint8_t *uid, uint8_t *uid_len);
```

UIDs are up to `RC522_MAX_UID_LEN` (7) bytes. Mapping a UID to a
behaviour is the caller's responsibility — see `thinkpack-nfc` for a
registry and NVS blob format.

## Consumers

- `packages/thinkpack/finderbox/` — NFC scan-to-chime firmware.

Ported from the self-contained driver in
`packages/games/nfc-scavenger-hunt/main/rc522_driver.{c,h}`, which
remains the reference implementation for the original scavenger-hunt
game and was the source for this component.

## Pin mapping

Board pin assignments live in `rc522_driver.c`:

- XIAO ESP32-S3 (default): SCK=7, MISO=8, MOSI=9, CS=5, RST=4.
- ESP32-S3 SuperMini (`-DBOARD_SUPERMINI`): SCK=12, MISO=13, MOSI=11,
  CS=10, RST=14.

See the consuming firmware's `WIRING.md` for the full wiring and any
board-specific notes.
