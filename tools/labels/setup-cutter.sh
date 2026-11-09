#!/usr/bin/env bash
# Install the inkscape-silhouette driver headless: a pinned checkout plus a venv
# with inkex, no Inkscape. Idempotent; re-run after bumping DRIVER_REV.
#
#   tools/labels/setup-cutter.sh            # installs into ~/.cache/inkscape-silhouette
#   SILHOUETTE_HOME=/elsewhere tools/labels/setup-cutter.sh
#
# inkex 1.4 declares PyGObject (GTK, for Inkscape's dialogs). sendto_silhouette.py
# never imports it, and building it needs cairo and gobject-introspection, so
# inkex goes in with --no-deps and its remaining runtime deps are listed here.
# On macOS the driver imports usb1 (libusb1) as well as pyusb, and both need the
# libusb C library at run time: `brew install libusb`. requirements.txt omits
# libusb1; install_osx_stage2.py lists it.
set -euo pipefail

DRIVER_REPO=https://github.com/fablabnbg/inkscape-silhouette
DRIVER_REV=f376a0836b53826e57f50d7ec44658663e629f05 # 2026-10-14
HOME_DIR=${SILHOUETTE_HOME:-$HOME/.cache/inkscape-silhouette}
SRC=$HOME_DIR/src
VENV=$HOME_DIR/venv

mkdir -p "$HOME_DIR"
if [ ! -d "$SRC/.git" ]; then
    git clone --quiet "$DRIVER_REPO" "$SRC"
fi
if [ "$(git -C "$SRC" rev-parse HEAD)" != "$DRIVER_REV" ]; then
    git -C "$SRC" fetch --quiet origin "$DRIVER_REV"
    git -C "$SRC" checkout --quiet --detach "$DRIVER_REV"
fi

if [ ! -x "$VENV/bin/python" ]; then
    uv venv --quiet --python 3.12 "$VENV"
fi
uv pip install --quiet --python "$VENV/bin/python" --no-deps "inkex==1.4.1"
uv pip install --quiet --python "$VENV/bin/python" \
    "lxml>=4.8,<6" "numpy>=1.21.5,<3" "cssselect>=1.2,<2" "tinycss2>=1.1.1,<2" \
    "packaging>=20.3" "pyparsing>=3.0.9" "pyserial>=3.4,<4" "scour>=0.37,<0.38" \
    "Pillow>=7" "pyusb>=1.2.1" "libusb1>=3" "xmltodict>=0.13" "matplotlib>=3.7.2"

"$VENV/bin/python" "$SRC/sendto_silhouette.py" --version
echo "driver: $SRC @ ${DRIVER_REV:0:12}"
echo "python: $VENV/bin/python"
