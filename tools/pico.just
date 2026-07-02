# Shared Raspberry Pi Pico SDK project configuration (RP2040/RP2350)
# Import this from project justfiles: import '../../../tools/pico.just'
#
# Requires these variables defined in your project justfile:
#   project_dir  — relative path from repo root (e.g., "packages/robotics/balancebot")
#   board        — PICO_BOARD value (e.g., "seeed_xiao_rp2350")
#
# Provides:
#   container_cmd    — docker or podman
#   compose_file     — path to shared docker-compose.yml
#   _detected_pico   — auto-detected RP2 USB CDC port (/dev/cu.usbmodem*)
#   _monitor_baud    — serial monitor baud rate (CDC ignores it; kept for parity)
#   require-port     — private recipe that fails if `port` is empty
#   _serial-monitor  — private recipe: pyserial monitor using `port`
#   _pico-build      — containerized cmake + ninja build (produces build/<name>.uf2)
#   _pico-clean      — remove the build directory
#   _pico-shell      — interactive container shell
#
# Flash/monitor run natively on the host (USB passthrough into containers is
# unreliable): `picotool load -f build/<name>.uf2` or BOOTSEL drag-drop.

# Container engine (docker or podman)
container_cmd := env("CONTAINER_CMD", "docker")

# Shared docker-compose.yml (relative to each project at packages/<domain>/<name>/)
compose_file := "../../../docker-compose.yml"

# Auto-detect RP2040/RP2350 USB CDC serial port (macOS: usbmodem)
_detected_pico := `ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true`

# Baud rate is ignored by USB CDC but pyserial requires a value
_monitor_baud := env("MONITOR_BAUD", "115200")

# Require a valid serial port for device recipes
[private]
require-port:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -z "{{port}}" ]; then
        echo "ERROR: No serial device found" >&2
        echo "  Is the board plugged in? Override with: PORT=/dev/... just monitor" >&2
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

# Build firmware (containerized cmake + ninja)
[private]
_pico-build:
    #!/usr/bin/env bash
    set -euo pipefail
    name="$(basename {{project_dir}})"
    echo "Building ${name} for {{board}}..."
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        pico-sdk \
        bash -c "cmake -B build -G Ninja -DPICO_BOARD={{board}} && ninja -C build"

# Remove build artifacts
[private]
_pico-clean:
    rm -rf build
    @echo "Clean complete"

# Interactive shell in the Pico SDK container
[private]
_pico-shell:
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        pico-sdk \
        /bin/bash
