#!/bin/bash
# SessionStart hook: print container-oriented dev context for Claude Code sessions.
#
# All ESP-IDF / Pico builds are containerized (see .claude/rules/containerized-builds.md);
# no local ESP-IDF install is required. This hook deliberately does NOT export a
# local IDF_PATH or advertise a local-install workflow — that contradicted the
# container-only architecture and pointed at recipes that do not exist.
set -e

echo "MCU Tinkering Lab — Development Environment"
echo "============================================"
echo ""

# Builds run in Docker; surface whether the engine is available.
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    echo "Container engine: $(docker --version)"
else
    echo "Container engine: Docker not found — builds are containerized, install Docker Desktop"
fi

echo ""
echo "Quick commands (run 'just --list' for all):"
echo "  just check-environment      - Verify Docker + serial port setup"
echo "  just docker-build           - Build the ESP-IDF dev image"
echo "  just list-projects          - List registered project modules"
echo "  just robocar::build-all     - Build the robocar (both controllers)"
echo "  just <module>::build        - Build any project (see: just --list --list-submodules)"
echo ""
echo "Slash Commands: /build, /flash, /develop, /debug"

exit 0
