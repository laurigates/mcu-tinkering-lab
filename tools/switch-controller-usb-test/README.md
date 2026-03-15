# Switch Pro Controller USB Protocol Testing Tools

Three tools for testing the Nintendo Switch Pro Controller USB protocol without
the build/flash/plug cycle:

| Tool | What it does | Hardware needed |
|------|-------------|-----------------|
| **switch_probe.py** | Talk to a real Pro Controller over USB | Pro Controller + USB-C cable |
| **switch_gadget.py** | PC emulates a Pro Controller via USB OTG | USB OTG port (Raspberry Pi, some laptops) |
| **switch_proxy.py** | ESP32-S3 handles USB, PC handles protocol via UART | ESP32-S3 + USB-UART adapter |

## Setup

```bash
cd tools/switch-controller-usb-test
uv sync
```

## Tool 1: switch_probe.py — Probe a Real Pro Controller

Connect a real Pro Controller via USB-C and send raw HID commands to it.

```bash
uv run python switch_probe.py           # Interactive REPL
uv run python switch_probe.py --scan    # List HID devices
uv run python switch_probe.py --auto    # Auto handshake + dump info
```

## Tool 2: switch_gadget.py — USB Gadget Emulator

Makes the PC appear as a Pro Controller to the Switch using Linux's configfs
USB gadget subsystem. All protocol logic runs in Python. **No firmware needed.**

**Requirements:**
- Linux with USB OTG/UDC hardware (Raspberry Pi 4/5, some laptops)
- Root access
- `dwc2` or similar UDC driver

**Raspberry Pi setup:**
```bash
# Add to /boot/config.txt:
dtoverlay=dwc2

# Load kernel modules:
sudo modprobe dwc2 libcomposite
```

**Usage:**
```bash
# Interactive mode
sudo uv run python switch_gadget.py

# Auto-setup and wait for Switch
sudo uv run python switch_gadget.py --auto

# List available UDCs
sudo uv run python switch_gadget.py --list-udc
```

**REPL commands:**
- `setup` — Create USB gadget (auto-detects UDC)
- `wait` — Wait for Switch handshake to complete
- `press a` / `release a` / `tap a` — Send button inputs
- `stick l 0 2048` — Set left stick position (0-4095, center=2048)
- `spi-set 6050 FF0000` — Override SPI flash data (e.g., body color to red)
- `record session.jsonl` — Record all traffic
- `teardown` — Remove USB gadget

**How it works:**
```
Switch --USB--> PC (USB gadget mode)
                |
          Python handles all
          protocol logic
```

## Tool 3: switch_proxy.py — ESP32-S3 USB Proxy

Flash a thin proxy firmware to the ESP32-S3 **once**. It handles only the USB
device layer (TinyUSB) and forwards raw HID bytes over UART to the PC. Protocol
logic runs in Python — iterate without reflashing.

**Hardware setup:**
```
Switch --USB--> ESP32-S3 --UART--> USB-UART adapter --USB--> PC
                  |                  (CP2102/CH340)
            Proxy firmware
            (flash once)
```

**ESP32-S3 UART pins:** GPIO43 (TX) / GPIO44 (RX), 921600 baud.

### Flash the proxy firmware (one time)

```bash
cd packages/esp32-projects/switch-usb-proxy
idf.py build
idf.py flash
```

### Run the PC-side client

```bash
# Interactive mode (auto-detects serial port)
uv run python switch_proxy.py

# Auto-connect and wait for Switch handshake
uv run python switch_proxy.py --auto --port /dev/ttyUSB0

# List serial ports
uv run python switch_proxy.py --list-ports
```

**REPL commands:**
- `connect /dev/ttyUSB0` — Connect to proxy firmware
- `auto on|off` — Toggle auto-respond to Switch commands
- `wait` — Wait for setup to complete
- `press a` / `release a` / `tap a` — Send button inputs
- `stick l 0 2048` — Set stick position
- `spi-set 6050 FF0000` — Override SPI flash data
- `spi-dump` — Show all emulated SPI flash entries
- `send-raw 81 01 ...` — Send raw HID report bytes
- `record session.jsonl` — Record all traffic

### UART Frame Format

```
[0xAA] [len] [direction] [report_data...] [checksum]

direction: 'S' (0x53) = Switch → PC
           'P' (0x50) = PC → Switch
checksum:  XOR of all payload bytes
```

## Linux udev rule (for probe tool)

```bash
# /etc/udev/rules.d/99-switch-pro.rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2009", MODE="0666"
```

Then reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

## Protocol Reference

See `packages/esp32-projects/xbox-switch-bridge/docs/switch-pro-controller-protocol.md`
for the full protocol documentation.
