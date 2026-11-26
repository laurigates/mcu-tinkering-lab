#!/usr/bin/env bash
# Install what the local carrier recipes need (tools/kicad.just, ADR-027):
#
#   - Freerouting 2.5.0 (the autorouter carrier-board runs), pinned by sha256,
#     to ~/.cache/freerouting/freerouting-2.5.0.jar;
#   - a Temurin 25 JRE for it (Freerouting 2.5.0 needs Java 25 or later; the
#     distribution JDKs are usually older), checked against the checksum the
#     Adoptium API publishes, to ~/.cache/freerouting/jre;
#   - KiCad 10.0.6, the version tools/kicad/vendor.py pins: on Ubuntu 24.04
#     from the KiCad PPA's Launchpad archive (needs sudo), pinned by sha256.
#     The PPA itself carries only the newest 10.0.x, and Launchpad answered
#     404 for the 10.0.6 .deb on 2026-11-12 and 2026-11-26 (#1146). With
#     --docker, or on Linux when that download fails and docker is present,
#     it instead writes `kicad-cli` and `kicad-python` wrappers that run the
#     official kicad/kicad:10.0.6-full image, pinned by digest, and prints
#     the environment the recipes need. Elsewhere it prints where to get it.
#
# Idempotent: what is already in place is checked and kept.
#
#   tools/kicad/setup.sh            # everything
#   tools/kicad/setup.sh --no-kicad # Freerouting and the JRE only
#   tools/kicad/setup.sh --docker   # KiCad from the Docker image
set -euo pipefail

KICAD_VERSION=10.0.6
FREEROUTING_VERSION=2.5.0
# Recorded when first downloaded (2026-10-18). A mismatch means the upstream
# file changed; check before re-pinning.
FREEROUTING_SHA256=f6f51bb02245e8e717f9359bd260cc9c5c0b1bc0acc8b7cb2cd5b8ffeb5de3c7
LAUNCHPAD=https://launchpad.net/~kicad/+archive/ubuntu/kicad-10.0-releases/+files
DEB_SUFFIX="${KICAD_VERSION}~ubuntu24.04.1"
declare -A DEB_SHA256=(
    ["kicad_${DEB_SUFFIX}_amd64.deb"]=a4920d3fc7b5719b8d5efa4ee24603a4313b50a53409bb5417f50770ce73926d
    ["kicad-symbols_${DEB_SUFFIX}_all.deb"]=7ba3be78336f3c11a1ffbdc00a6010772c2a54cdac5a249a6c18c67e10921577
    ["kicad-footprints_${DEB_SUFFIX}_all.deb"]=4b9751a5bcaf4eefb56c061b4c7387a3e06e213a4ff4c5c6ff77ce9e39c9a8e1
)

# kicad/kicad:10.0.6-full on Docker Hub, read 2026-11-26. "-full" carries the
# 3D models tools/enclosure exports in the board's STEP.
KICAD_IMAGE=kicad/kicad:10.0.6-full@sha256:e36e29b5f4a638a1a7238eba3d2fcbc8e061ffed1e99a8f39cb722706c94a58e
BIN="${HOME}/.local/bin"

CACHE="${HOME}/.cache/freerouting"
JAR="${CACHE}/freerouting-${FREEROUTING_VERSION}.jar"
JRE="${CACHE}/jre"
WITH_KICAD=1
DOCKER=0
[ "${1:-}" = "--no-kicad" ] && WITH_KICAD=0
[ "${1:-}" = "--docker" ] && DOCKER=1

sha256() {
    if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1
    else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

fetch() { # url dest sha256
    local tmp="$2.part"
    curl -fsSL -o "$tmp" "$1"
    local got
    got=$(sha256 "$tmp")
    if [ "$got" != "$3" ]; then
        rm -f "$tmp"
        echo "FAIL: $1: sha256 $got, expected $3" >&2
        exit 1
    fi
    mv "$tmp" "$2"
}

mkdir -p "$CACHE"

# --- Freerouting -------------------------------------------------------------
if [ -f "$JAR" ] && [ "$(sha256 "$JAR")" = "$FREEROUTING_SHA256" ]; then
    echo "ok: $JAR"
else
    fetch "https://github.com/freerouting/freerouting/releases/download/v${FREEROUTING_VERSION}/freerouting-${FREEROUTING_VERSION}.jar" \
        "$JAR" "$FREEROUTING_SHA256"
    echo "installed: $JAR"
fi

# --- Java 25 for Freerouting -------------------------------------------------
java_major() { "$1" -version 2>&1 | sed -n 's/.*version "\([0-9]*\).*/\1/p' | head -1; }
if [ -x "$JRE/bin/java" ] && [ "$(java_major "$JRE/bin/java")" -ge 25 ]; then
    echo "ok: $JRE (Java $(java_major "$JRE/bin/java"))"
else
    case "$(uname -s)" in
        Linux) os=linux ;;
        Darwin) os=mac ;;
        *) echo "FAIL: no Temurin build for $(uname -s)" >&2; exit 1 ;;
    esac
    case "$(uname -m)" in
        x86_64 | amd64) arch=x64 ;;
        aarch64 | arm64) arch=aarch64 ;;
        *) echo "FAIL: no Temurin build for $(uname -m)" >&2; exit 1 ;;
    esac
    meta=$(curl -fsSL "https://api.adoptium.net/v3/assets/latest/25/hotspot?architecture=${arch}&image_type=jre&os=${os}&vendor=eclipse")
    read -r link sum < <(python3 -c '
import json, sys
pkg = json.load(sys.stdin)[0]["binary"]["package"]
print(pkg["link"], pkg["checksum"])' <<<"$meta")
    fetch "$link" "${CACHE}/jre.tar.gz" "$sum"
    rm -rf "$JRE" "${CACHE}/jre.unpack"
    mkdir -p "${CACHE}/jre.unpack"
    tar -xzf "${CACHE}/jre.tar.gz" -C "${CACHE}/jre.unpack"
    home=$(find "${CACHE}/jre.unpack" -path '*/bin/java' -type f | head -1)
    mv "$(dirname "$(dirname "$home")")" "$JRE"
    rm -rf "${CACHE}/jre.unpack" "${CACHE}/jre.tar.gz"
    echo "installed: $JRE (Java $(java_major "$JRE/bin/java"))"
fi

# --- KiCad -------------------------------------------------------------------
[ "$WITH_KICAD" = 1 ] || exit 0

# Wrappers that run KiCad from $KICAD_IMAGE with the repo, /tmp and the
# Freerouting cache mounted at their own paths, as the calling user.
docker_kicad() {
    command -v docker >/dev/null || { echo "FAIL: docker not found" >&2; exit 1; }
    docker pull -q "$KICAD_IMAGE" >/dev/null
    local repo share="${HOME}/.cache/kicad-${KICAD_VERSION}" tool cmd
    repo=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
    mkdir -p "$BIN"
    for tool in kicad-cli kicad-python; do
        cmd=kicad-cli
        [ "$tool" = kicad-python ] && cmd=python3
        {
            echo '#!/bin/sh'
            echo "# Written by tools/kicad/setup.sh --docker: KiCad ${KICAD_VERSION} from the image."
            echo 'exec docker run --rm -i -u "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONPATH -e FREEROUTING_JAR \'
            echo "    -e PATH=${JRE}/bin:/usr/local/bin:/usr/bin:/bin \\"
            echo "    -v ${repo}:${repo} -v /tmp:/tmp -v ${CACHE}:${CACHE} -w \"\$PWD\" \\"
            echo "    ${KICAD_IMAGE} ${cmd} \"\$@\""
        } >"$BIN/$tool"
        chmod +x "$BIN/$tool"
    done
    # tools/kicad/library.py compares the vendored libraries with an installed
    # KiCad's: give it the image's.
    if [ ! -d "$share/footprints" ]; then
        local cid
        cid=$(docker create "$KICAD_IMAGE")
        mkdir -p "$share"
        docker cp "$cid:/usr/share/kicad/symbols" "$share/symbols"
        docker cp "$cid:/usr/share/kicad/footprints" "$share/footprints"
        docker rm "$cid" >/dev/null
    fi
    echo "installed: $BIN/kicad-cli and $BIN/kicad-python ($("$BIN/kicad-cli" version))"
    echo "add to the shell (the recipes read these):"
    echo "  export PATH=\"$BIN:\$PATH\" KICAD_PYTHON=$BIN/kicad-python \\"
    echo "    KICAD_SYMBOL_DIR=$share/symbols KICAD_FOOTPRINT_DIR=$share/footprints \\"
    echo "    KICAD10_3DMODEL_DIR=/usr/share/kicad/3dmodels FREEROUTING_JAR=$JAR"
    exit 0
}

[ "$DOCKER" = 1 ] && docker_kicad
if command -v kicad-cli >/dev/null && [ "$(kicad-cli version 2>/dev/null)" = "$KICAD_VERSION" ]; then
    echo "ok: kicad-cli $KICAD_VERSION"
    exit 0
fi
if [ "$(uname -s)" = Darwin ]; then
    echo "KiCad $KICAD_VERSION: install the macOS build from https://downloads.kicad.org/kicad/macos/explore/stable"
    exit 0
fi
# shellcheck disable=SC1091
. /etc/os-release
if [ "${ID:-}" != ubuntu ] || [ "${VERSION_CODENAME:-}" != noble ] || [ "$(uname -m)" != x86_64 ]; then
    echo "KiCad $KICAD_VERSION: no pinned package for ${PRETTY_NAME:-this system} ($(uname -m));"
    echo "install it from https://www.kicad.org/download/ and check \`kicad-cli version\`."
    exit 0
fi
debs="${CACHE}/kicad-debs"
mkdir -p "$debs"
for deb in "${!DEB_SHA256[@]}"; do
    if [ ! -f "$debs/$deb" ] || [ "$(sha256 "$debs/$deb")" != "${DEB_SHA256[$deb]}" ]; then
        if ! (fetch "$LAUNCHPAD/$deb" "$debs/$deb" "${DEB_SHA256[$deb]}"); then
            command -v docker >/dev/null || exit 1
            echo "Launchpad did not serve $deb (#1146); using the Docker image" >&2
            docker_kicad
        fi
    fi
done
SUDO=""
[ "$(id -u)" = 0 ] || SUDO=sudo
$SUDO apt-get update -qq
# Without recommends: kicad-packages3d (3D models) is several GB and no recipe
# reads it.
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y -qq --no-install-recommends \
    "$debs/kicad_${DEB_SUFFIX}_amd64.deb" \
    "$debs/kicad-symbols_${DEB_SUFFIX}_all.deb" \
    "$debs/kicad-footprints_${DEB_SUFFIX}_all.deb"
echo "installed: kicad-cli $(kicad-cli version)"
