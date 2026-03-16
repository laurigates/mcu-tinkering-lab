# Switch USB Proxy

A thin ESP32-S3 firmware that bridges USB HID traffic to/from a PC over UART. The ESP32-S3 presents as a Nintendo Switch Pro Controller over USB; all protocol logic runs on the PC in Python.

## How It Works

```
┌──────────┐    USB HID     ┌──────────────┐    UART 921600    ┌───────────┐
│ Nintendo │ ──────────────→│  ESP32-S3    │──────────────────→│    PC     │
│ Switch   │ ←──────────────│  USB Proxy   │←──────────────────│  Python   │
└──────────┘                └──────────────┘                   └───────────┘
                             (flash once)                    (switch_proxy.py)
```

Flash the proxy firmware **once**. Iterate on protocol responses in Python without reflashing.

## Hardware Required

- **Waveshare ESP32-S3-Zero** (or any ESP32-S3 with native USB-OTG)
- **USB-UART adapter** (CP2102, CH340, etc.) connected to GPIO43 (TX) / GPIO44 (RX)
- **USB-C cable** from ESP32-S3 to Switch dock

## Building

Requires ESP-IDF v5.4+ (builds run in Docker).

```bash
just build         # Build firmware
just flash         # Flash to ESP32-S3
```

Or without `just`:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash
```

## Usage

1. Flash the proxy firmware to the ESP32-S3
2. Connect the USB-UART adapter (GPIO43 TX, GPIO44 RX)
3. Connect the ESP32-S3's USB-C to the Switch dock
4. Run the PC-side client:

```bash
cd tools/switch-controller-usb-test
uv sync
uv run python switch_proxy.py --auto
```

See [`tools/switch-controller-usb-test/README.md`](../../../tools/switch-controller-usb-test/README.md) for full PC-side tool documentation.

## UART Frame Format

```
[0xAA] [len] [direction] [report_data...] [checksum]

direction: 'S' (0x53) = Switch → PC
           'P' (0x50) = PC → Switch
checksum:  XOR of all payload bytes (direction + report_data)
```

## Related Projects

- [xbox-switch-bridge](../xbox-switch-bridge/) — Full BLE→USB bridge firmware
- [switch-controller-usb-test](../../../tools/switch-controller-usb-test/) — PC-side Python protocol tools
- [Switch Pro Controller Protocol](../xbox-switch-bridge/docs/switch-pro-controller-protocol.md) — Protocol reference

## References

- [Nintendo Switch Reverse Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering) — Pro Controller protocol docs
- [ESP-IDF TinyUSB](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/usb_device.html) — USB device support
