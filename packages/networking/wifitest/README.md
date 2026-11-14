# wifitest

A bare-minimum WiFi access point for checking that an ESP32-S3 board's radio
works: it starts a WPA2 SoftAP with fixed settings, logs each station that
joins or leaves, and does nothing else. There is no station mode, no web
server and no application logic.

## Hardware

- **Board**: an ESP32-S3 board with a USB-Serial-JTAG port (the web flasher
  lists it as "ESP32-S3 DevKit"). `sdkconfig.defaults` turns PSRAM off and
  sets no flash size; the `flash` recipe writes the image with a 4 MB,
  80 MHz DIO header (`_s3-flash` in `tools/esp32-idf.just`).
- **Peripherals**: none. The firmware assigns no GPIO, so there is no
  WIRING.md.

## Access point

These values are compiled in (`main/main.c`); there is nothing to configure:

| Setting | Value |
|---------|-------|
| SSID | `esp32s3-test` |
| Password | `testpass1234` (WPA2-PSK) |
| Channel | 6, HT20 bandwidth |
| Max stations | 4 |
| Country | `FI`, channels 1-13 |
| TX power | `esp_wifi_set_max_tx_power(78)`, in 0.25 dBm units (19.5 dBm by the source comment); the log prints the value read back |

AMPDU RX and TX are disabled in `sdkconfig.defaults`, which calls it a known
ESP32-S3 AP-visibility workaround. ADR-028 records that workaround as
unverified ([ADR-028](../../../docs/decisions/ADR-028-teuvo-connectivity-resilience.md),
"SoftAP captive portal").

## Build and flash

Released builds can be flashed from the browser; see
[Flash from the browser](../../../README.md#flash-from-the-browser).

To build from source (containerized ESP-IDF, so Docker and `just` are the
only requirements):

```bash
just wifitest::build          # build in the ESP-IDF container, check the .bin
just wifitest::flash          # flash over USB (port auto-detected, or PORT=...)
just wifitest::monitor        # serial monitor with reset (Ctrl-C to stop)
just wifitest::flash-monitor  # flash, then monitor
```

`just --list wifitest` shows the rest (`clean`, `menuconfig`, `shell`,
`reset`).

## What to expect

`app_main` routes stdout and stderr to USB-Serial-JTAG, so `monitor` shows the
log over the same cable used for flashing. After boot it logs the app version,
then a line of the form
`AP started: SSID=esp32s3-test channel=6 tx_power=...dBm bw=HT20 country=FI`
and the password. Each client then logs `Station connected: AID=<n>` or
`Station disconnected: AID=<n>`.

The table and log lines above are read from source; they have not been
checked against a running board for this README.

## Project structure

```
wifitest/
├── main/
│   ├── main.c          — SoftAP setup and station event logging
│   └── CMakeLists.txt
├── flasher.json        — web flasher entry
├── justfile            — build, flash and monitor recipes
├── sdkconfig.defaults  — target, WiFi buffers, AMPDU off, Bluetooth and PSRAM off
└── CMakeLists.txt      — project definition; version from version.txt
```
