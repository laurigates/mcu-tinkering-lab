# Xbox → Switch Controller Bridge

An ESP32-S3 firmware that bridges an **Xbox Series wireless controller** (connected via Bluetooth LE) to a **Nintendo Switch** (connected via USB, emulating a wired Pro Controller).

## How It Works

```
┌──────────────┐    Bluetooth LE    ┌──────────────┐    USB HID     ┌──────────────┐
│ Xbox Series  │ ──────────────────→│  ESP32-S3    │──────────────→│  Nintendo    │
│ Controller   │    (Bluepad32)     │  Bridge      │  (TinyUSB)    │  Switch      │
└──────────────┘                    └──────────────┘               └──────────────┘
```

1. **BLE Host** (Bluepad32): Scans for and connects to the Xbox controller
2. **Button Mapper**: Translates Xbox inputs → Switch Pro Controller format (swaps A/B, X/Y)
3. **USB Device** (TinyUSB): Emulates a wired Switch Pro Controller (VID: 0x057E, PID: 0x2009)

## Hardware Required

- **Waveshare ESP32-S3-Zero** (recommended — tiny, native USB, 4MB flash, 2MB PSRAM)
- **Xbox Series wireless controller** (the one with BLE support — has the Share button)
- **USB-C to USB-A cable/adapter** from ESP32-S3-Zero to Switch dock

Any ESP32-S3 board with native USB-OTG will work. See [WIRING.md](WIRING.md) for other supported boards.

> **Note:** Standard ESP32 (WROOM/WROVER) will NOT work — it lacks native USB support. You need the S3 or S2.

## Button Mapping

| Xbox | Switch | Position |
|------|--------|----------|
| A | B | Bottom |
| B | A | Right |
| X | Y | Left |
| Y | X | Top |
| LB | L | Left shoulder |
| RB | R | Right shoulder |
| LT | ZL | Left trigger |
| RT | ZR | Right trigger |
| View | - (Minus) | Left menu |
| Menu | + (Plus) | Right menu |
| Xbox | Home | Center |
| Share | Capture | Center-left |

Analog sticks and D-pad are mapped 1:1.

## Building

Requires ESP-IDF v5.4+.

```bash
# From repo root
cd packages/input-gaming/xbox-switch-bridge

# Set target and build
idf.py set-target esp32s3
idf.py build

# Flash (ESP32-S3-Zero uses /dev/ttyACM0, hold BOOT+RESET first)
idf.py -p /dev/ttyACM0 flash monitor

# Or use the Makefile
make build
make flash-monitor PORT=/dev/ttyACM0
```

## Usage

1. Flash the firmware to your ESP32-S3
2. Connect ESP32-S3 via USB to your Switch dock
3. Turn on your Xbox controller and put it in **pairing mode** (hold the Pair button until the Xbox button flashes rapidly)
4. The ESP32-S3 will automatically connect and start bridging inputs
5. The Switch should recognize it as a "Pro Controller"

## Architecture

```
main/main.c                         # Application entry, bridge task loop
components/
├── bluepad32_host/                  # BLE gamepad host (wraps Bluepad32)
│   ├── bluepad32_host.c
│   └── include/bluepad32_host.h
├── button_mapper/                   # Xbox → Switch input translation
│   ├── button_mapper.c
│   └── include/button_mapper.h
└── switch_pro_usb/                  # Switch Pro Controller USB emulation
    ├── switch_pro_usb.c
    └── include/switch_pro_usb.h
```

## Dependencies

Managed via ESP-IDF Component Manager (`main/idf_component.yml`):

- **Bluepad32** (>=4.1): BLE gamepad host library
- **esp_tinyusb** (>=1.4.0): USB device stack for ESP32-S3

## References

- [Bluepad32](https://github.com/ricardoquesada/bluepad32) - Bluetooth gamepad library
- [Nintendo Switch Reverse Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering) - Pro Controller protocol docs
- [ESP-IDF TinyUSB](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/usb_device.html) - USB device support
