#!/usr/bin/env bash
# Clone Bluepad32 at its pinned tag and prepare btstack as an ESP-IDF component.
#
#   tools/fetch-bluepad32.sh <dest>      clone into <dest>, e.g. external/bluepad32
#   tools/fetch-bluepad32.sh --version   print the pinned tag
#
# Callers: xbox-switch-bridge's `fetch-deps` recipe (gamepad-synth reaches the
# checkout through that recipe or a relative symlink to it) and the bluepad32
# step of .github/workflows/_ci-build-esp32.yml. build-firmware.yml keeps its
# own copy of the step, so a release build does not depend on this file at the
# tag; tools/test_workflows.py (BluepadTagTest) fails when its tag differs from
# BP32_TAG below (#1082).
#
# Safe to re-run: the clone is skipped when <dest>/src exists, and the btstack
# copy when the component's src/ exists.
set -euo pipefail

BP32_TAG="release_v3.10.3"
BP32_REPO="https://github.com/ricardoquesada/bluepad32.git"

usage() {
    echo "usage: $0 <dest> | --version" >&2
    exit 2
}

[ $# -eq 1 ] || usage
case "$1" in
    --version)
        echo "$BP32_TAG"
        exit 0
        ;;
    -* | "") usage ;;
esac

BP32_DIR="$1"
BTSTACK_ROOT="$BP32_DIR/external/btstack"
BTSTACK_COMP="$BTSTACK_ROOT/port/esp32/components/btstack"

if [ ! -d "$BP32_DIR/src" ]; then
    echo "Cloning bluepad32 $BP32_TAG..."
    git clone --depth 1 --branch "$BP32_TAG" "$BP32_REPO" "$BP32_DIR"
    git -C "$BP32_DIR" submodule update --init --depth 1 external/btstack
fi

# btstack's component CMakeLists uses paths relative to the component
# directory, so the sources it builds are copied into it.
if [ ! -d "$BTSTACK_COMP/src" ]; then
    echo "Preparing btstack component..."
    for dir in src 3rd-party/bluedroid 3rd-party/hxcmod-player \
        3rd-party/lwip/dhcp-server 3rd-party/lc3-google \
        3rd-party/md5 3rd-party/micro-ecc 3rd-party/yxml \
        platform/freertos platform/lwip; do
        mkdir -p "$BTSTACK_COMP/$(dirname "$dir")"
        cp -r "$BTSTACK_ROOT/$dir" "$BTSTACK_COMP/$dir"
    done
    # platform/embedded: only these four files
    mkdir -p "$BTSTACK_COMP/platform/embedded"
    for f in hal_time_ms.h hal_uart_dma.h hci_dump_embedded_stdout.h hci_dump_embedded_stdout.c; do
        cp "$BTSTACK_ROOT/platform/embedded/$f" "$BTSTACK_COMP/platform/embedded/"
    done
fi
echo "Dependencies ready"
