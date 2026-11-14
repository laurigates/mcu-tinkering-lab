# it-troubleshooter

A USB device for network troubleshooting on a Waveshare ESP32-S3-Zero. Plugged
into a target computer it enumerates as a composite USB device: a HID
keyboard and a CDC serial port. Command output pasted into the CDC port is
sent to the Claude API over WiFi, and the command Claude suggests next is
typed into the target through the keyboard, until Claude answers `DONE`.

Requirements, phases and roadmap are in the
[PRD](docs/prds/it-troubleshooter.md); design decisions in the
[ADRs](docs/adrs/README.md); architecture, hardware constraints and
component APIs in [CLAUDE.md](CLAUDE.md).

## Hardware

- **Board**: Waveshare ESP32-S3-Zero (4 MB flash, 2 MB quad PSRAM, per
  `sdkconfig.defaults`).
- **Status LED**: the board's WS2812 on GPIO21; the colour of each mode is
  in [CLAUDE.md](CLAUDE.md#components).
- **Optional debug adapter**: once the firmware starts TinyUSB, the
  USB-Serial-JTAG port leaves the bus, so the log is read from a USB-UART
  adapter on GPIO43 (TX) / GPIO44 (RX) at 115200 baud
  ([ADR-0006](docs/adrs/0006-external-uart-debug.md)). No other external
  wiring.

## Build and flash

Released builds can be flashed from the browser; see
[Flash from the browser](../../../README.md#flash-from-the-browser). A
release binary carries no credentials (see Configuration), so it stops
before the Claude loop.

To build from source (containerized ESP-IDF, so Docker and `just` are the
only requirements):

```bash
just troubleshooter::setup    # create main/credentials.h from the template
just troubleshooter::build    # build in the ESP-IDF container
just troubleshooter::flash    # flash over USB (port auto-detected, or PORT=...)
just troubleshooter::deploy   # build, then flash
just troubleshooter::monitor  # serial monitor on the USB port, with reset
```

`just --list troubleshooter` shows the rest (`clean`, `menuconfig`, `shell`,
`reset`, `detect-port`, `info`). `monitor` reads the USB port, which per
ADR-0006 carries the log only until TinyUSB starts; use the UART adapter
above for anything later.

## Configuration

Credentials are compile-time constants in `main/credentials.h` (gitignored):
`WIFI_SSID`, `WIFI_PASS`, `CLAUDE_API_KEY` and `CLAUDE_MODEL`. Create it
with `just troubleshooter::setup`, which copies
[`main/credentials.h.example`](main/credentials.h.example) and leaves an
existing file alone, then edit it.

If the file is missing at build time, `CMakeLists.txt` writes a stub
`main/credentials.h` with every value empty; this is how CI and the web
flasher build. After that, `setup` reports the file as existing, so edit the
stub or delete it and run `setup` again. Nothing sets credentials at
runtime. With empty WiFi values the USB keyboard and serial port still come
up, but `app_main` stops at WiFi (init failure, or the end of its 30-second
connect wait), sets the LED to the error mode and returns without starting
the diagnostic loop (read from `main/main.c`, not checked on hardware).

## Project structure

```
it-troubleshooter/
├── main/main.c          — init sequence and the CDC-to-Claude diagnostic loop
├── components/
│   ├── usb_composite/   — TinyUSB HID keyboard + CDC
│   ├── claude_client/   — Claude API requests
│   ├── wifi_manager/    — STA connection with retry
│   ├── state_machine/   — NVS-persisted workflow state
│   └── status_led/      — WS2812 driver
├── docs/                — PRD, ADRs, PRPs
├── justfile             — build, flash and monitor recipes
└── sdkconfig.defaults   — TinyUSB classes, PSRAM, TLS, partition table
```
