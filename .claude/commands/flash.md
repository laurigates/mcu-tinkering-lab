---
description: Flash firmware to an ESP32 device
argument-hint: "[project-name] [port]"
allowed-tools: Bash(just:*), Bash(esptool*), Bash(ls:*)
---

# Flash Firmware to ESP32 Device

Flash compiled firmware to an ESP32 device. Flashing runs natively (not in container).

## Parse Arguments

$ARGUMENTS may contain:
- Project name (first word)
- Serial port (second word, or auto-detected)

## Available Projects

- `robocar-main` or `main`, `robocar-cam` or `cam`
- `webserver`, `i2s-audio` or `audio`, `telegram`
- `kids-audio`, `xbox`, `wifitest`

## Port Auto-Detection

Ports are auto-detected by `tools/esp32.just`:
- USB-serial adapters (CH340/CP2102): `ls /dev/cu.usbserial-*`
- ESP32-S3 native USB: detected by Espressif VID `0x303a`
- Override with: `PORT=/dev/... just <module>::flash`

## Flash Commands

Based on $ARGUMENTS:

- `robocar-main`: `just robocar::flash-main`
- `robocar-cam`: `just robocar::flash-cam`
- `webserver`: `just webserver::flash`
- `i2s-audio`: `just i2s-audio::flash`
- `telegram`: `just telegram::flash`
- `kids-audio`: `just kids-audio::flash`
- `xbox`: `just xbox::flash`
- `wifitest`: `just wifitest::flash`

## ESP32-CAM Note

For ESP32-CAM projects (robocar-cam, webserver, i2s-audio, telegram):
- Remind the user: "Connect GPIO0 to GND before flashing"
- After successful flash: "Disconnect GPIO0 from GND and reset the device"

## Error Handling

If flashing fails:
1. Check if port exists and is accessible
2. Suggest checking USB connection
3. For permission errors, suggest: `sudo chmod 666 /dev/xxx`
