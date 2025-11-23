#!/bin/bash
# SessionStart hook: Set up ESP-IDF environment for Claude Code sessions
# This hook runs when Claude Code starts and exports environment variables

set -e

# Default IDF_PATH if not set
IDF_PATH="${IDF_PATH:-$HOME/repos/esp-idf}"

# Export environment to CLAUDE_ENV_FILE if available
if [ -n "$CLAUDE_ENV_FILE" ]; then
    # Export IDF_PATH
    echo "export IDF_PATH=\"$IDF_PATH\"" >> "$CLAUDE_ENV_FILE"

    # Export common build settings
    echo "export SDKCONFIG_DEFAULTS=\"sdkconfig.defaults\"" >> "$CLAUDE_ENV_FILE"
fi

# Print session context
echo "MCU Tinkering Lab - Development Environment"
echo "============================================"
echo ""

# Check ESP-IDF installation
if [ -d "$IDF_PATH" ]; then
    VERSION=$(cd "$IDF_PATH" && git describe --tags 2>/dev/null || echo "unknown")
    echo "ESP-IDF: $VERSION"
    echo "Path: $IDF_PATH"
else
    echo "ESP-IDF: Not installed"
    echo "Run 'make setup-idf' to install automatically"
fi

# Show available projects
echo ""
echo "Quick Commands:"
echo "  make setup-idf       - Install ESP-IDF"
echo "  make robocar-build-all - Build robocar"
echo "  make check-environment - Verify setup"
echo ""
echo "Slash Commands: /setup-idf, /build, /flash, /develop, /debug"

exit 0
