# Shared credentials.h bootstrap helper
# Import this from project justfiles that have a main/credentials.h.example:
#   import '../../../tools/esp32-credentials.just'
#
# Provides:
#   _credentials hint="..." (private) — copy main/credentials.h.example to
#                                        main/credentials.h if missing
#
# Usage in a project justfile:
#
#   # Use default hint
#   [group: "setup"]
#   credentials: _credentials
#
#   # Override hint
#   [group: "setup"]
#   credentials: (_credentials "edit WIFI_SSID, WIFI_PASSWORD, GEMINI_API_KEY")
#
#   # Or chain into build
#   [group: "build"]
#   build: credentials _idf-build

# Copy main/credentials.h.example to main/credentials.h if missing
[private]
_credentials hint="edit it with your WiFi and API credentials":
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -f main/credentials.h ]; then
        echo "main/credentials.h already exists — no changes made"
    else
        cp main/credentials.h.example main/credentials.h
        echo "Created main/credentials.h — {{hint}}"
    fi
