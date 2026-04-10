# Shared ESPHome project recipes
# Import this from ESPHome project justfiles: import '../../../tools/esphome.just'
#
# Requires these variables defined in your project justfile:
#   device_name     — ESPHome device name (e.g., "audiobook-player")
#   config_file     — YAML config file (e.g., "audiobook-player.yaml")
#   secrets_file    — secrets file name (e.g., "secrets.yaml")
#   secrets_example — secrets example file (e.g., "secrets.yaml.example")
#
# Provides:
#   Public recipes (never overridden):
#     install, validate, compile, wireless, dashboard, clean, status
#   Private helpers (compose into public recipes as needed):
#     _esphome-config  — create secrets file from template
#     _esphome-upload  — compile and upload via USB (with secrets check)
#     _esphome-logs    — view device logs

# Install ESPHome
[group: "setup"]
install:
    pip install --upgrade esphome

# Create secrets file from template — private helper for custom messages
[private]
_esphome-config:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f {{secrets_file}} ]; then
        cp {{secrets_example}} {{secrets_file}}
        echo "Created {{secrets_file}} — edit it with your credentials"
    else
        echo "{{secrets_file}} already exists"
    fi

# Compile and upload via USB — private helper for custom upload logic
[private]
_esphome-upload:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -f {{secrets_file}} ]; then
        echo "Error: {{secrets_file}} not found — run 'just config' first"
        exit 1
    fi
    esphome run {{config_file}}

# View device logs — private helper for custom log logic
[private]
_esphome-logs:
    esphome logs {{config_file}}

# Validate ESPHome configuration
[group: "build"]
validate:
    esphome config {{config_file}}

# Compile firmware
[group: "build"]
compile:
    esphome compile {{config_file}}

# Upload via WiFi OTA
[group: "flash"]
wireless:
    esphome run {{config_file}} --device {{device_name}}.local

# Open ESPHome dashboard (access at http://localhost:6052)
[group: "monitor"]
dashboard:
    esphome dashboard .

# Clean build files
[confirm("Remove .esphome/ build directory?")]
[group: "build"]
clean:
    rm -rf .esphome/ *.bin *.elf *.map

# Show project status
[group: "info"]
status:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Device: {{device_name}}"
    echo "Config: {{config_file}}"
    if [ -f {{secrets_file}} ]; then
        echo "Status: Configured"
    else
        echo "Status: Not configured — run 'just config'"
    fi
