# Shared ESP-IDF containerized build recipes
# Import this from ESP-IDF project justfiles: import '../../../tools/esp32-idf.just'
#
# Requires these variables defined in your project justfile:
#   project_dir  — relative path from repo root (e.g., "packages/robocar/main")
#   target       — ESP32 chip target (e.g., "esp32", "esp32s3")
#
# Provides (all private — compose into public recipes in your project justfile):
#   _idf-build       — containerized set-target + build
#   _idf-clean       — containerized fullclean
#   _idf-menuconfig  — containerized menuconfig
#   _idf-shell       — interactive container shell
#   _idf *args       — run any idf.py command in the container
#
# Also re-exports everything from esp32.just (container_cmd, compose_file,
# port detection, require-port, _serial-monitor).
#
# Example usage in a project justfile:
#
#   import '../../../tools/esp32-idf.just'
#
#   project_dir := "packages/<domain>/my-project"
#   target := "esp32"
#
#   [group: "build"]
#   build: _idf-build
#
#   [group: "build"]
#   clean: _idf-clean
#
#   # Or with pre-build steps:
#   [group: "build"]
#   build: credentials _idf-build

import 'esp32.just'

# _idf-build-checked, _esp32-flash and _s3-flash below take the compiled app
# basename (no .bin extension) as their `bin` parameter. Projects define their
# own bin_name variable and pass it, e.g.:
#   bin_name := "thinkpack-boombox"
#   build: (_idf-build-checked bin_name)
#   flash: (_s3-flash bin_name)
# Passing it as a recipe argument (not a shared variable) sidesteps just's
# import-default-vs-module-override conflict and keeps the .bin name explicit at
# the call site. Flash baud reads ${BAUD:-460800} at the bash level.

# Run any idf.py command in the container
[private]
_idf *args:
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        idf.py {{args}}

# Build firmware (containerized)
[private]
_idf-build:
    #!/usr/bin/env bash
    set -euo pipefail
    name="$(basename {{project_dir}})"
    echo "Building ${name}..."
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        bash -c "idf.py set-target {{target}} && idf.py build"

# Clean build artifacts (containerized)
[private]
_idf-clean:
    #!/usr/bin/env bash
    set -euo pipefail
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        idf.py fullclean
    echo "Clean complete"

# Open configuration menu (containerized)
[private]
_idf-menuconfig:
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        idf.py menuconfig

# Interactive shell in ESP-IDF container
[private]
_idf-shell:
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        /bin/bash

# Validated containerized build: filters build output to warnings/errors/size and
# fails fast if the app binary is missing. Requires bin_name to match the CMake
# project() name. Opt-in — compose as `build: _idf-build-checked`.
[private]
_idf-build-checked bin:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building {{bin}}..."
    {{container_cmd}} compose -f {{compose_file}} run --rm \
        -w /workspace/{{project_dir}} \
        esp-idf \
        bash -c "idf.py set-target {{target}} && idf.py build" 2>&1 \
        | grep -E "warning:|error:|ninja: build stopped|binary size|Build complete" \
        || true
    if [ ! -f build/{{bin}}.bin ]; then
        echo "FAILED — rerun verbose:" >&2
        echo "  {{container_cmd}} compose -f {{compose_file}} run --rm -w /workspace/{{project_dir}} esp-idf idf.py build" >&2
        exit 1
    fi
    echo "Build OK ({{bin}}.bin)"

# BEFORE COMPOSING EITHER SHARED FLASH RECIPE, CHECK ALL FOUR:
#
#   1. CHIP        _esp32-flash writes the bootloader at 0x1000, _s3-flash at
#                  0x0. Wrong one = a board that does not boot. (Loud.)
#   2. FLASH SIZE  _s3-flash HARDCODES --flash-size 4MB into the bootloader
#                  header. Declaring CONFIG_ESPTOOLPY_FLASHSIZE above 4MB and
#                  composing it anyway understates the part, and anything past
#                  4 MB in the partition table becomes unreachable. (Silent.)
#                  _esp32-flash uses `detect`, so it is exempt.
#   3. OTA DATA    NEITHER recipe writes ota_data_initial.bin. If the partition
#                  table has an otadata segment, flashing leaves whatever OTA
#                  state was there — so a board that has OTA'd to ota_1 keeps
#                  booting ota_1 while you flash new firmware into ota_0.
#                  Flashes clean, runs the old image. (Silent, and the nastiest.)
#   4. APP OFFSET  Both hardcode 0x10000. A table that puts the app elsewhere
#                  gets its app written to the wrong place. (Silent.)
#
# Fail any one and keep the flash recipe INLINE with explicit offsets;
# packages/robocar/unified/justfile is the worked example. Three of the four
# fail silently, which is why "does this look like a standard layout?" is the
# wrong test — run the mechanical one:
#
#     python3 tools/check-flash-recipes.py
#
# It also runs as a pre-commit hook, so this is enforced rather than remembered.
# Nothing else covers these recipes: CI builds firmware via esp-idf-ci-action,
# never through the justfile, and flashing is host-native. After editing either
# recipe, confirm every consumer still expands as intended with
# `PORT=/dev/ttyDUMMY just --dry-run <module>::flash`.

# Flash ESP32 firmware natively (bootloader @0x1000, flash-size detect).
# Standard single-app layout. Uses bin_name + baud.
[private]
_esp32-flash bin: require-port
    #!/usr/bin/env bash
    set -euo pipefail
    esptool --chip esp32 -p {{port}} -b "${BAUD:-460800}" \
        --before default-reset --after hard-reset \
        write-flash --flash-mode dio --flash-size detect \
        0x1000 build/bootloader/bootloader.bin \
        0x8000 build/partition_table/partition-table.bin \
        0x10000 build/{{bin}}.bin
    echo "Flashed {{bin}} OK"

# Flash ESP32-S3 firmware natively (bootloader @0x0, 4MB @80MHz).
# Standard single-app layout. Uses bin_name + baud.
[private]
_s3-flash bin: require-port
    #!/usr/bin/env bash
    set -euo pipefail
    esptool -s --chip esp32s3 -p {{port}} -b "${BAUD:-460800}" \
        --before default-reset --after hard-reset \
        write-flash --flash-mode dio --flash-size 4MB --flash-freq 80m \
        0x0 build/bootloader/bootloader.bin \
        0x8000 build/partition_table/partition-table.bin \
        0x10000 build/{{bin}}.bin
    echo "Flashed {{bin}} OK"

# Reset an ESP32-S3 board via USB-Serial-JTAG CDC control signals.
[private]
_s3-reset: require-port
    "{{tools_dir}}/esp32s3-reset.sh" "{{port}}"

# Serial monitor for an ESP32-S3 via USB-Serial-JTAG (does a CDC reset first).
[private]
_s3-monitor: require-port
    "{{tools_dir}}/esp32s3-monitor.sh" "{{port}}"
