---
name: project-health
description: Audit project completeness against the monorepo standard checklist (docs, config, registration)
argument-hint: "[project-path|all]"
user-invocable: true
allowed-tools: Read, Grep, Glob, Bash(just:*), Bash(ls:*)
---

## Task

Audit project(s) for completeness against the monorepo standard checklist.

- If `$ARGUMENTS` is a project path → audit that single project
- If `$ARGUMENTS` is `all` or empty → audit all projects under `packages/esp32-projects/`

## Checklist

For each project, check these items. Report pass/warn/fail for each.

### 1. README.md
- **Pass**: File exists and is >10 lines
- **Warn**: File exists but is very short (<10 lines)
- **Fail**: Missing

### 2. CLAUDE.md
- **Pass**: File exists and is >20 lines
- **Warn**: File exists but is very short
- **Fail**: Missing

### 3. WIRING.md (hardware projects only)
- **Pass**: File exists
- **Skip**: Project has no GPIO usage (simulation, docs hub)
- **Fail**: Project uses hardware but has no WIRING.md

To determine if a project uses hardware: check for `gpio_config`, `#define.*GPIO`, `ledc_`, `i2c_`, `spi_`, `uart_` in source files.

### 4. justfile
- **Pass**: File exists and imports `tools/esp32.just`
- **Warn**: File exists but doesn't import shared config
- **Fail**: Missing (for ESP-IDF projects)
- **Skip**: ESPHome or non-ESP-IDF projects

### 5. Root justfile registration
- **Pass**: Project appears as `mod` in root `justfile`
- **Fail**: Not registered

Check by grepping root `justfile` for the project directory name.

### 6. sdkconfig.defaults (ESP-IDF only)
- **Pass**: File exists
- **Fail**: Missing for ESP-IDF project

### 7. version.txt
- **Pass**: File exists
- **Fail**: Missing

### 8. .gitignore coverage
- **Pass**: Project has own `.gitignore` OR root `.gitignore` covers common patterns (build/, sdkconfig)
- **Warn**: No coverage found

### 9. Credentials handling (WiFi/API projects only)
- **Pass**: `credentials.h.example` or `secrets.yaml.example` exists
- **Skip**: Project doesn't use WiFi or API credentials
- **Fail**: Source references credentials but no example file exists

Check by grepping for `#include "credentials.h"`, `#include "secrets.h"`, `wifi_config` in source.

### 10. mDNS configuration (WiFi STA projects only)
- **Pass**: `mdns` in CMakeLists.txt REQUIRES and `CONFIG_MDNS_ENABLED` in sdkconfig.defaults
- **Skip**: Project doesn't use WiFi STA mode
- **Fail**: WiFi STA project without mDNS

Check by grepping for `esp_wifi_set_mode.*WIFI_MODE_STA` or `wifi_init_sta` in source.

## Output Format

### Single Project

```
## Project Health: <project-name>

| Check | Status | Notes |
|-------|--------|-------|
| README.md | PASS | 85 lines |
| CLAUDE.md | FAIL | Missing |
| ... | ... | ... |

Score: 8/10 checks passed

Suggested actions:
- Run `/project-claude-md <path>` to generate CLAUDE.md
- Run `/wiring-doc <path>` to generate WIRING.md
```

### All Projects

```
## Monorepo Health Summary

| Project | README | CLAUDE | WIRING | justfile | registered | sdkconfig | version | gitignore | creds | mDNS | Score |
|---------|--------|--------|--------|----------|------------|-----------|---------|-----------|-------|------|-------|
| robocar-main | PASS | FAIL | ... | ... | ... | ... | ... | ... | ... | ... | 7/10 |
| ... | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... |

Projects needing attention:
- <project>: missing CLAUDE.md, WIRING.md
```
