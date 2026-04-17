# Shared ESP-IDF project configuration
# Import this from project justfiles: import '../../../tools/esp32.just'
#
# Provides:
#   container_cmd    — docker or podman
#   compose_file     — path to shared docker-compose.yml
#   tools_dir        — absolute path to the monorepo tools/ directory
#   _detected_serial — auto-detected USB-serial adapter port (CH340/CP2102/FTDI)
#   _detected_s3     — auto-detected ESP32-S3 USB-Serial-JTAG port
#   _monitor_baud    — serial monitor baud rate (default 115200, override in project)
#   require-port     — private recipe that fails if `port` is empty
#   _serial-monitor  — private recipe: pyserial monitor using `port` and `_monitor_baud`

# Container engine (docker or podman)
container_cmd := env("CONTAINER_CMD", "docker")

# Shared docker-compose.yml (relative to each project at packages/esp32-projects/<name>/)
compose_file := "../../../docker-compose.yml"

# Absolute path to monorepo tools/ directory (for helper scripts)
tools_dir := justfile_directory() / "../../.." / "tools"

# Auto-detect USB-serial adapter (CH340/CP2102/FTDI)
_detected_serial := `ls /dev/cu.usbserial-* 2>/dev/null | head -1 || true`

# Auto-detect ESP32-S3 USB-Serial-JTAG by Espressif VID (0x303a)
_detected_s3 := `ioreg -r -c IOUSBHostDevice -l 2>/dev/null | grep -A 30 '"idVendor" = 12346' | grep IODialinDevice | head -1 | sed 's/.*"\(\/dev\/[^"]*\)".*/\1/' | sed 's/tty\./cu./' || true`

# Auto-detect CP2102/FTDI USB-UART adapter (for secondary UART connections)
_detected_uart := `ls /dev/cu.usbserial-* 2>/dev/null | head -1 || true`

# Default serial monitor baud rate (override with MONITOR_BAUD env var)
_monitor_baud := env("MONITOR_BAUD", "115200")

# Require a valid serial port for device recipes
[private]
require-port:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -z "{{port}}" ]; then
        echo "ERROR: No serial device found" >&2
        echo "  Is the board plugged in? Override with: PORT=/dev/... just flash" >&2
        exit 1
    fi

# Serial monitor via pyserial (Ctrl-C to stop)
[private]
_serial-monitor: require-port
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Monitoring {{port}} @ {{_monitor_baud}} baud — Ctrl-C to stop"
    exec uvx --from pyserial python3 -c "
    import serial, sys, signal
    signal.signal(signal.SIGINT, lambda *a: sys.exit(0))
    s = serial.Serial('{{port}}', {{_monitor_baud}}, timeout=1)
    try:
        while True:
            data = s.read(4096)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    except serial.SerialException:
        print('\nDevice disconnected', file=sys.stderr)
    "
