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
