# cam-i2s-audio

**Status: placeholder.** The firmware does not yet use the camera, WiFi or
I2S. `main/main.c` prints four "Initializing ..." lines and then
`Main loop running... waiting for tasks.` every 10 seconds; every feature is
a `TODO` comment. The release on the web flasher is this placeholder.

The plan written in those comments is a loop on the AI Thinker ESP32-CAM:
capture an image, send it to a server over HTTP, receive audio back, and play
it through a MAX98357A I2S amplifier.

## Hardware

- **Board**: AI Thinker ESP32-CAM (classic ESP32; `target := "esp32"` in the
  justfile).
- **Peripherals**: none in use. The MAX98357A is named only in comments
  (`main/main.c`, `sdkconfig.defaults`); no camera or I2S pins are assigned,
  so there is no WIRING.md yet.

## Build and flash

Released builds can be flashed from the browser; see
[Flash from the browser](../../../README.md#flash-from-the-browser).

To build from source (containerized ESP-IDF, so Docker and `just` are the
only requirements):

```bash
just i2s-audio::build          # build in the ESP-IDF container
just i2s-audio::flash          # flash with esptool (GPIO0 must be tied to GND)
just i2s-audio::monitor        # serial monitor (Ctrl-C to stop)
just i2s-audio::flash-monitor  # flash, then monitor
just i2s-audio::develop        # build, flash and monitor
just i2s-audio::info           # target, port and the GPIO0 programming steps
```

To flash: connect GPIO0 to GND, power on or reset the board, run `flash`,
then disconnect GPIO0 and reset (`just i2s-audio::info` prints the same
steps). `just --list i2s-audio` shows the rest (`clean`, `menuconfig`,
`shell`).

## Configuration

None. The project has no `credentials.h`; nothing reads WiFi or API
credentials yet. `sdkconfig.defaults` enables WiFi and HTTPS for the HTTP
client, ahead of the code that will use them.

## Project structure

```
cam-i2s-audio/
├── main/
│   ├── main.c          — placeholder app_main with TODOs
│   └── CMakeLists.txt  — registers main.c; the future REQUIRES list is commented out
├── flasher.json        — web flasher entry
├── justfile            — build, flash and monitor recipes
├── sdkconfig.defaults  — WiFi and HTTPS client switches
└── CMakeLists.txt      — project definition; version from version.txt
```
