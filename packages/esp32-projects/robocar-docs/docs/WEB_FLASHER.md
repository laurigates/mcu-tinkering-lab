# Web Flasher

Flash robocar firmware directly from your browser using the [ESP Web Tools](https://esphome.github.io/esp-web-tools/) web flasher. No toolchain installation required.

**Live page**: `https://laurigates.github.io/mcu-tinkering-lab/`

## Requirements

- **Browser**: Google Chrome or Microsoft Edge (version 89+). Firefox and Safari are not supported (no Web Serial API).
- **Android**: Chrome 117+ with a USB-C OTG adapter.
- **USB connection** to the target board (see board-specific notes below).

## Supported Boards

### Main Controller (Heltec WiFi LoRa 32 V1)

Connect via the onboard micro-USB port. No special boot mode is needed.

### Camera Module (ESP32-CAM AI Thinker)

The ESP32-CAM does not have onboard USB. You need an external USB-to-serial adapter (FTDI, CP2102, or CH340).

**Wiring:**

| Adapter | ESP32-CAM |
|---------|-----------|
| TX | U0R (RX) |
| RX | U0T (TX) |
| GND | GND |
| 5V | 5V |

**Enter flash mode:**

1. Connect GPIO0 to GND (some boards have a button for this)
2. Press the RESET button (or power cycle)
3. Release GPIO0 after flashing begins

## WiFi Configuration

Pre-built firmware does not include WiFi credentials. After flashing, you need to configure WiFi:

### Option A: Serial Console (available now)

1. After flashing, click **Logs & Console** in ESP Web Tools
2. The device serial output will appear
3. Use the serial command interface to set WiFi credentials

### Option B: Improv WiFi (planned, see [#151](https://github.com/laurigates/mcu-tinkering-lab/issues/151))

Once implemented, ESP Web Tools will automatically show a WiFi configuration dialog after flashing. No manual serial interaction needed.

## How It Works

The web flasher is deployed to GitHub Pages automatically on every firmware release:

1. `build-firmware.yml` builds both firmware targets using ESP-IDF
2. Bootloader, partition table, and app binaries are extracted
3. ESP Web Tools manifests are generated with correct flash offsets
4. Everything is deployed to GitHub Pages alongside the flasher HTML page

### Flash Layout

Both boards use the same partition layout (4MB flash):

```
0x1000   bootloader.bin       (bootloader)
0x8000   partition-table.bin  (partition table)
0xF000   ota_data_initial.bin (OTA boot selector)
0x12000  app.bin              (firmware, max 1.8MB)
```

The firmware includes dual OTA partitions for over-the-air updates after initial flashing.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No compatible device found" | Check USB cable (must support data, not charge-only) |
| Browser shows no install button | Use Chrome or Edge; Firefox/Safari not supported |
| ESP32-CAM won't flash | Ensure GPIO0 is held LOW during reset |
| Android can't see device | Grant USB permission when prompted; try a different OTG adapter |
| Flash succeeds but device won't connect to WiFi | Configure credentials via serial console (see above) |
| Device boot-loops after flash | OTA rollback should recover previous firmware. If fresh flash, re-flash via web flasher |
