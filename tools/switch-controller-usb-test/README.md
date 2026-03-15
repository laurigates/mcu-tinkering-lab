# Switch Pro Controller USB Protocol Tester

Interactive tool for probing a real Nintendo Switch Pro Controller over USB.
Connect the controller via USB-C, then use this tool to send raw HID commands,
record responses, and map out protocol behavior.

## Setup

```bash
cd tools/switch-controller-usb-test
uv sync
```

## Usage

### Interactive REPL

```bash
uv run python switch_probe.py
```

Commands in the REPL:
- `handshake` — Run the full USB handshake sequence (0x80 01→04)
- `info` — Request device info (subcmd 0x02)
- `spi <addr_hex> <len>` — Read SPI flash (subcmd 0x10)
- `subcmd <id_hex> [arg_hex...]` — Send arbitrary subcommand
- `raw <hex_bytes>` — Send raw output report bytes
- `poll [seconds]` — Poll input reports for N seconds (default 5)
- `buttons` — Live button/stick display (Ctrl+C to stop)
- `dump-cal` — Dump all calibration data from SPI flash
- `dump-colors` — Read body/button/grip colors
- `record <filename>` — Start recording all traffic to file
- `stop` — Stop recording
- `replay <filename>` — Replay recorded commands
- `scan` — List all HID devices (find your controller)
- `help` — Show command help
- `quit` — Exit

### Non-interactive modes

```bash
# Scan for HID devices
uv run python switch_probe.py --scan

# Run handshake + dump device info
uv run python switch_probe.py --auto

# Record full session to file
uv run python switch_probe.py --record session.jsonl

# Replay a recorded session
uv run python switch_probe.py --replay session.jsonl
```

## Requirements

- Python 3.11+
- A Nintendo Switch Pro Controller connected via USB-C
- On Linux: `sudo` or udev rules for HID access

### Linux udev rule

```bash
# /etc/udev/rules.d/99-switch-pro.rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="057e", ATTRS{idProduct}=="2009", MODE="0666"
```

Then reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

## Protocol Reference

See `packages/esp32-projects/xbox-switch-bridge/docs/switch-pro-controller-protocol.md`
for the full protocol documentation.
