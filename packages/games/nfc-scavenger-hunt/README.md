# NFC Scavenger Hunt

A voice-guided toy treasure hunt on an ESP32-S3. Each hidden toy carries an
NFC tag. Tapping a newly found toy on the RC522 reader plays a chime, then
the board asks Gemini text-to-speech for a spoken clue to the next toy and
plays it through a MAX98357A I2S amplifier. Progress survives power cycles,
and once every toy is found the clue is replaced by a spoken celebration.

Goals and requirements: [PRD-006](../../../docs/requirements/PRD-006-nfc-scavenger-hunt.md).

## Hardware

- **Board**: XIAO ESP32-S3 in every build today. The source also carries an
  ESP32-S3 SuperMini pin set that nothing selects; see
  [WIRING.md](WIRING.md#board).
- **Peripherals**: RC522 NFC reader (SPI), MAX98357A I2S amplifier, a push
  button and a status LED.

Pins, the two board variants and the hazards are in [WIRING.md](WIRING.md).
The RC522 driver is the shared
[`thinkpack-rc522`](../../components/thinkpack-rc522/README.md) component.

## Build and flash

Released builds can be flashed from the browser; see
[Flash from the browser](../../../README.md#flash-from-the-browser). A
release binary carries no credentials (see Configuration), so it plays only
the built-in sounds.

To build from source (containerized ESP-IDF, so Docker and `just` are the
only requirements):

```bash
just nfc-scavenger-hunt::build          # build in the ESP-IDF container, check the .bin
just nfc-scavenger-hunt::flash          # flash bootloader, partition table and app
just nfc-scavenger-hunt::monitor        # serial monitor with reset (Ctrl-C to stop)
just nfc-scavenger-hunt::flash-monitor  # flash, then monitor
just nfc-scavenger-hunt::info           # target, binary, port and mDNS name
```

`just --list nfc-scavenger-hunt` shows the rest (`clean`, `menuconfig`,
`shell`, `reset`).

## Configuration

**Credentials** are compile-time constants in `main/credentials.h`
(gitignored): `WIFI_SSID`, `WIFI_PASSWORD` and `GEMINI_API_KEY`. The project
has no recipe for it; copy the template and edit it:

```bash
cp packages/games/nfc-scavenger-hunt/main/credentials.h.example packages/games/nfc-scavenger-hunt/main/credentials.h
```

If the file is missing at build time, `CMakeLists.txt` writes a stub
`main/credentials.h` with every value empty; this is how CI and the web
flasher build. The stub then counts as the real file, so edit it (or
delete it and copy the template) before the next build. Nothing sets
credentials at runtime: when WiFi does not connect, the board logs a
warning, plays the error tone and carries on with the built-in sounds only.

**Tags** are listed in `main/tag_config.h`. The UIDs there are placeholders.
To use real tags, flash, tap each one, copy the UID from the
`Unknown tag:` log line into `TAG_REGISTRY` with its name and hint, and
reflash (the steps are in the comment above `TAG_REGISTRY`). Each entry's
`hint` is the clue spoken after that toy is found. `MAX_TAGS` is 10.

## Behaviour

- The reader is polled every 300 ms; the same tag is ignored for 3 s after
  a scan.
- A new find plays a local chime at once, then fetches the clue from
  `gemini-2.5-flash-preview-tts` (voice `Kore`). An already-found tag and an
  unknown tag each get their own local sound and no request.
- Found tags are a bit mask in NVS (namespace `scavenger`, key `found_mask`),
  so progress survives a reset. Holding the button for 2 s clears it.
- After WiFi connects the board answers as `nfc-scavenger-hunt.local`.
- The status LED patterns are listed in [WIRING.md](WIRING.md#status-led).

This section is read from source and has not been checked on a board for
this README.

## Project structure

```
nfc-scavenger-hunt/
├── main/
│   ├── main.c                  — init, scan task, find handling, reset button
│   ├── game_logic.c            — tag lookup and NVS progress
│   ├── tag_config.h            — tag registry (UIDs, names, hints)
│   ├── gemini_tts.c            — Gemini TTS request, base64 audio decode
│   ├── prompt_builder.c        — clue and celebration prompts
│   ├── local_sounds.c          — synthesised chimes and tones
│   ├── i2s_audio.c             — MAX98357A output and playback task
│   ├── status_led.c            — LED patterns
│   ├── wifi_manager.c          — STA connection, 5 retries
│   └── credentials.h.example   — credentials template
├── WIRING.md                   — pins and hazards
├── justfile                    — build, flash and monitor recipes
├── sdkconfig.defaults          — PSRAM, WiFi, TLS, partition table, 4 MB flash
└── CMakeLists.txt              — credentials stub, version, thinkpack-rc522 path
```
