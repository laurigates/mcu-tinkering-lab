# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESPHome-based RFID audiobook player for kids. Scan picture cards with RFID tags to trigger Home Assistant automations that play audiobooks. Physical play/pause buttons provide tactile control.

**Hardware:** TTGO LoRa32 V2.0 ESP32 (with OLED & LoRa), RC522 RFID reader (SPI), two buttons (GPIO32/GPIO33), tilt switch (GPIO27) for deep sleep wake, piezo buzzer (GPIO4), vibration motor (GPIO26)

**Integration:** Device sends events to Home Assistant via ESPHome API. Home Assistant automations map RFID tag UIDs to media playback actions.

## Build and Development Commands

### Using Makefile (Recommended)

```bash
# Initial setup
make install          # Install ESPHome
make config          # Create secrets.yaml from template

# Development workflow
make compile         # Compile firmware only
make upload          # Compile and upload via USB (auto-detects serial port)
make wireless        # Upload via WiFi OTA after initial USB flash
make logs            # View device logs in real-time

# Utilities
make validate        # Validate YAML configuration
make clean           # Remove build artifacts
make status          # Show project status
make pins            # Display pin assignment reference
```

### Direct ESPHome Commands

```bash
# Compile only
esphome compile audiobook-player.yaml

# Upload via USB (requires port detection)
esphome run audiobook-player.yaml --device /dev/cu.usbserial-*

# Upload via WiFi OTA
esphome run audiobook-player.yaml --device audiobook-player.local

# View logs
esphome logs audiobook-player.yaml
```

## Critical Hardware Notes

### GPIO Pin Constraints

**TTGO LoRa32 V2.0 Reserved Pins (LoRa & OLED):**
- LoRa SX1276: GPIO5 (SCK), GPIO18 (CS), GPIO19 (MISO), GPIO26 (DIO0), GPIO27 (MOSI)
- OLED SSD1306: GPIO21 (SDA), GPIO22 (SCL)
- Built-in blue LED: GPIO25

**⚠️ CRITICAL:** The onboard LoRa chip shares the VSPI bus. Even though LoRa is not used, **GPIO5/18/19/27 cause SPI bus contention** if used for other SPI devices. Use HSPI (GPIO12-15) for RC522.

**Strapping pins:** GPIO0, GPIO2, GPIO5, GPIO12, GPIO15 (can cause boot issues if externally pulled). GPIO12/15 are OK for RC522 if not pulled at boot.

**Safe GPIOs for peripherals:** GPIO4, GPIO13, GPIO14, GPIO25, GPIO32-GPIO33

### RC522 RFID Module

- **3.3V ONLY** - Do not connect to 5V or module will be damaged
- **Must use HSPI bus** - VSPI pins conflict with onboard LoRa SX1276 chip
- Uses HSPI: CLK=GPIO14, MISO=GPIO35, MOSI=GPIO13, CS=GPIO15 (no RST needed)
- **Do NOT use GPIO12 for MISO** - strapping pin prevents flashing when high
- GPIO35 is input-only, perfect for MISO
- Update interval: 500ms (configurable in YAML)

### Power Considerations

- TTGO LoRa32 V2.0 has JST connector for LiPo battery
- Built-in TP4054 charging circuit when powered via USB-C
- Status LED on GPIO25 (built-in blue LED, inverted logic)
- Onboard 0.96" OLED display (currently unused, available for future enhancements)

### Deep Sleep Power Management

**Tilt switch (GPIO27):** Wakes ESP32 from deep sleep when device is picked up/tilted
- Sleep mode: ~10µA power draw (~29 days on 18650 battery)
- Awake mode: ~80mA power draw (~17 hours on 18650 battery)
- Auto-sleep: Device enters deep sleep after 2 minutes of inactivity
- Wake time: 3-5 seconds to reconnect WiFi and become ready

**Important behaviors:**
- During deep sleep, device appears "unavailable" in Home Assistant (expected)
- RFID scans during WiFi reconnection (first 3-5 seconds after wake) won't reach Home Assistant
- OTA updates automatically prevent deep sleep to avoid interruption
- Deep sleep disabled during OTA to ensure update completes successfully

## Architecture

### Event Flow

1. **RFID Tag Scanned** → RC522 detects tag → ESPHome fires `on_tag` trigger
2. **ESPHome Actions:**
   - Updates `last_rfid_tag` text sensor
   - Sends `tag_scanned` event to Home Assistant (native HA tag system)
   - Sends `esphome.audiobook_triggered` custom event with tag_id
   - Blinks status LED (300ms)
3. **Home Assistant** receives events → Automation matches tag_id → Triggers media playback

### Button Events

Buttons send custom events (`esphome.audiobook_control`) with action field:
- Green button (GPIO32): `action: play`
- Red button (GPIO33): `action: pause`

Home Assistant automations listen for these events and control media players.

### Configuration Structure

- `audiobook-player.yaml` - Main ESPHome config (device, sensors, events)
- `secrets.yaml` - WiFi credentials, API keys, passwords (gitignored)
- `secrets.yaml.example` - Template for secrets
- `home-assistant-example.yaml` - Reference automations for Home Assistant

## Common Development Tasks

### Reading RFID Tag UIDs

1. Upload firmware: `make upload`
2. Monitor logs: `make logs`
3. Scan RFID tags near the RC522 module
4. Copy tag UIDs from logs (format: `XX-XX-XX-XX`)
5. Use UIDs in Home Assistant automations

### Modifying Pin Assignments

When changing GPIO pins:
1. Update `audiobook-player.yaml` configuration
2. Update `WIRING.md` documentation (diagrams and pin tables)
3. Update `Makefile` pins target reference
4. Verify pin is not a strapping pin (GPIO0, GPIO2, GPIO5, GPIO12, GPIO15)

### Troubleshooting Upload Issues

**"Error resolving IP address"**: Device not on WiFi yet or mDNS not working
- Solution: Use `make upload` (auto-detects USB port) or specify port manually

**"No ESP32 device found on USB"**: Serial port not detected
- Check USB cable is data-capable (not charge-only)
- macOS: Look for `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`
- Linux: Look for `/dev/ttyUSB*` or `/dev/ttyACM*`

**GPIO strapping pin warning**: Using boot-sensitive GPIO
- Avoid GPIO0, GPIO2, GPIO5, GPIO12, GPIO15 for peripherals with pull resistors

### Testing Without Home Assistant

1. Upload firmware: `make upload`
2. Watch logs: `make logs`
3. Scan RFID tags - should see "Tag scanned: XX-XX-XX-XX" messages
4. Press buttons - should see event logs even without HA connected
5. Status LED should blink on tag detection

## ESPHome Integration Points

### Text Sensors
- `last_rfid_tag` - Stores last scanned RFID UID (or empty when removed)

### Binary Sensors
- `tilt_switch` - GPIO27 (wakes from deep sleep, active low, internal pullup)
- `play_button` - GPIO32 (active low, internal pullup)
- `pause_button` - GPIO33 (active low, internal pullup)

### Events Sent to Home Assistant
- `tag_scanned` - Native Home Assistant tag event
- `esphome.audiobook_triggered` - Custom event with device_id and tag_id
- `esphome.audiobook_control` - Button press events with action field

### API Configuration
- Encrypted API connection (key in secrets.yaml)
- OTA updates enabled (password in secrets.yaml)
- Fallback AP: "Audiobook-Player" if WiFi fails
