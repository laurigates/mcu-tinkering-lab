# Wiring Guide - Xbox Switch Bridge

## No Wiring Required!

This project uses the ESP32-S3's **built-in** peripherals:

- **Bluetooth LE**: Built into the ESP32-S3 chip (no external antenna needed for short range)
- **USB OTG**: Uses the native USB pins on the ESP32-S3

## Connections

```
ESP32-S3 DevKit                          Nintendo Switch
┌─────────────────┐                     ┌─────────────┐
│                  │                     │             │
│  USB-C (OTG) ───┼──── USB Cable ────→ │ Dock USB    │
│                  │                     │             │
│  [Bluetooth] ←──┼─ ─ ─ ─ (air) ─ ─ →│             │
│                  │                     └─────────────┘
└─────────────────┘
         ↑
     Xbox Series
     Controller
    (Bluetooth LE)
```

## Important Notes

### USB Port Selection
- On ESP32-S3 DevKits with **two USB-C ports**: use the one labeled **USB** (not UART)
- The UART port is for programming/debugging; the USB port is the OTG interface that connects to the Switch

### ESP32-S3 DevKit Variants
| Board | USB OTG Port | Notes |
|-------|-------------|-------|
| ESP32-S3-DevKitC-1 | USB connector | Single USB, shared UART/OTG |
| ESP32-S3-DevKitM-1 | USB connector | Check silkscreen labels |
| LilyGo T-Display S3 | USB-C | Works well, has built-in display |
| Seeed XIAO ESP32-S3 | USB-C | Compact form factor |

### Power
- The ESP32-S3 draws power from the Switch dock's USB port
- No external power supply needed during normal operation
- For initial flashing, connect to your computer's USB port instead
