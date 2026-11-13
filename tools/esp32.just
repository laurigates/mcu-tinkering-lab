# Shared ESP-IDF project configuration
# Project justfiles import '../../../tools/esp32-idf.just', which imports this
# file; they do not import it themselves.
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
#   _serial-monitor-on mon_port mon_baud — private recipe: the same monitor on
#                      any port and baud, with no port check of its own

# Container engine (docker or podman)
container_cmd := env("CONTAINER_CMD", "docker")

# Shared docker-compose.yml, as an absolute path built from this file's
# directory (tools/). A path relative to the project only held at
# packages/<domain>/<name>/; a nested project such as
# packages/robotics/robocar/bringup/ is one level deeper.
compose_file := parent_directory(source_directory()) / "docker-compose.yml"

# Absolute path to monorepo tools/ directory (for helper scripts).
#
# source_directory() is the directory of *this* file (tools/), which is the
# answer regardless of how just was invoked. justfile_directory() is the
# directory of the ROOT justfile, so the old `justfile_directory()/../../../
# tools` only resolved correctly when a recipe was run from inside the package
# directory; run as a module from the repo root — `just <mod>::monitor`, the
# documented way — it produced <repo>/../../../tools and every helper-script
# recipe (_s3-reset, _s3-monitor) failed with "No such file or directory".
tools_dir := source_directory()

# Auto-detect USB-serial adapter (CH340/CP2102/FTDI)
_detected_serial := `ls /dev/cu.usbserial-* 2>/dev/null | head -1 || true`

# Auto-detect ESP32-S3 USB-Serial-JTAG by Espressif VID (0x303a), through the
# one copy of the ioreg pipeline in detect-esp32s3-port.sh. Every recipe of every
# importing module evaluates this at load, so it must print nothing and succeed
# when no board is attached: --quiet, stderr dropped, `|| true`. shell() rather
# than a backtick, because a backtick runs in the importing module's directory
# and the script's path has to come from this file's source_directory().
_detected_s3 := shell('"$1" --quiet 2>/dev/null || true', source_directory() / "detect-esp32s3-port.sh")

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
_serial-monitor: require-port (_serial-monitor-on port _monitor_baud)

# The pyserial monitor on a given port and baud. It checks no port itself:
# a second adapter (a UART monitor) composes it behind its own port check,
# since require-port checks `port`, which is empty when only that adapter is
# plugged in.
[private]
_serial-monitor-on mon_port mon_baud:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Monitoring {{mon_port}} @ {{mon_baud}} baud — Ctrl-C to stop"
    exec uvx --from pyserial python3 -c "
    import serial, sys, signal
    signal.signal(signal.SIGINT, lambda *a: sys.exit(0))
    s = serial.Serial('{{mon_port}}', {{mon_baud}}, timeout=1)
    try:
        while True:
            data = s.read(4096)
            if data:
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    except serial.SerialException:
        print('\nDevice disconnected', file=sys.stderr)
    "
