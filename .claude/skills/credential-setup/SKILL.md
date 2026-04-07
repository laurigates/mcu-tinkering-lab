---
name: credential-setup
description: Standardize credential handling for an ESP32 project (credentials.h.example, .gitignore, pre-commit protection)
argument-hint: "<project-path>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob
---

## Task

Standardize credential handling for the project at `$1` (relative path under `packages/esp32-projects/`).

## Standard Patterns

### ESP-IDF Projects

Use `credentials.h` + `credentials.h.example`:

```c
// credentials.h.example — Template for project credentials
// Copy to credentials.h and fill in your values:
//   cp credentials.h.example credentials.h

#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// WiFi Configuration
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// API Keys (uncomment and fill in as needed)
// #define API_KEY "your-api-key"

#endif // CREDENTIALS_H
```

### ESPHome Projects

Use `secrets.yaml` + `secrets.yaml.example`:

```yaml
# secrets.yaml.example — Template for project secrets
# Copy to secrets.yaml and fill in your values:
#   cp secrets.yaml.example secrets.yaml

wifi_ssid: "your-wifi-ssid"
wifi_password: "your-wifi-password"
api_encryption_key: "generate-with-esphome"
ota_password: "your-ota-password"
```

## Process

1. **Detect project type** (ESP-IDF or ESPHome) by checking for `CMakeLists.txt` vs `.yaml` config
2. **Scan source** for credential usage:
   - `#include "credentials.h"` or `#include "secrets.h"` or `#include "wifi_config.h"`
   - `WIFI_SSID`, `WIFI_PASSWORD`, `API_KEY`, etc.
3. **Check current state**:
   - Does a `.example` file exist?
   - Is the credential file gitignored?
   - Are pre-commit hooks protecting it?
4. **Fix non-conforming patterns**:
   - `wifi_config.h` / `wifi_config_example.h` → rename to `credentials.h` / `credentials.h.example`
   - `secrets.h` / `secrets.h.example` → rename to `credentials.h` / `credentials.h.example`
   - Update `#include` statements in source to match
5. **Ensure protection**:
   - Verify `.gitignore` covers `credentials.h` and `secrets.yaml`
   - Verify root `.pre-commit-config.yaml` blocks credential files

## Migration Table

| Current File | Standard Name | Action |
|-------------|--------------|--------|
| `wifi_config.h` | `credentials.h` | Rename, update includes |
| `wifi_config_example.h` | `credentials.h.example` | Rename |
| `secrets.h` | `credentials.h` | Rename, update includes |
| `secrets.h.example` | `credentials.h.example` | Rename |
| `credentials.h.template` | `credentials.h.example` | Rename |
| `credentials.h.example` | (already correct) | No action |

## Output

Report what was found and what was changed:
- Current credential pattern detected
- Files renamed/created
- .gitignore status
- Pre-commit protection status
- Source files updated (if includes were changed)
