---
name: register-project
description: Register a project in the monorepo — justfile mod, CI matrix, standard file inventory
argument-hint: "<project-name>"
user-invocable: true
allowed-tools: Read, Write, Edit, Grep, Glob, Bash(just:*)
---

## Task

Register the project `$1` in the monorepo infrastructure.

## Process

### 1. Verify Project Exists

Check that `packages/<domain>/$1/` exists and has the basic structure:
- `CMakeLists.txt` or ESPHome `.yaml` file
- `main/` directory (for ESP-IDF projects)
- `justfile`

### 2. Register in Root justfile

Add a `mod` line to the root `justfile` if not already present:

```just
mod project-name 'packages/<domain>/project-name'
```

Place it alphabetically among the existing `mod` declarations. Use the justfile recipe naming convention (lowercase-kebab-case).

### 3. Verify CI Matrix

Check `.github/workflows/esp32-build.yml` for the project matrix. If the project should be built in CI:
- Verify it appears in the build matrix
- Or note that it needs to be added manually (don't auto-edit workflow files without confirmation)

### 4. Check Standard File Inventory

Report which standard files exist and which are missing:

| File | Status |
|------|--------|
| `justfile` | exists / missing |
| `CMakeLists.txt` | exists / missing |
| `sdkconfig.defaults` | exists / missing |
| `version.txt` | exists / missing |
| `README.md` | exists / missing |
| `CLAUDE.md` | exists / missing |
| `WIRING.md` | exists / missing / n/a |
| `.gitignore` | exists / covered by root |

### 5. Suggest Follow-up Skills

For missing files, suggest the appropriate skill:
- Missing README → `/project-readme $1`
- Missing CLAUDE.md → `/project-claude-md $1`
- Missing WIRING.md → `/wiring-doc $1`
- Missing sdkconfig.defaults → `/sdkconfig-audit $1`
- Missing credentials setup → `/credential-setup $1`

## Output

```
## Project Registration: <project-name>

### Registration
- Root justfile: ADDED (mod line inserted)
- CI matrix: Present / NEEDS MANUAL ADD

### File Inventory
| File | Status |
|------|--------|
| ... | ... |

### Suggested Next Steps
- Run `/project-claude-md <name>` to generate CLAUDE.md
- ...
```
