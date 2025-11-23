---
description: Install or update ESP-IDF development framework
allowed-tools: Bash(make:*), Bash(git:*)
---

# Setup ESP-IDF Development Framework

Help the user set up or update the ESP-IDF development framework.

## Steps

1. First check the current ESP-IDF status:
   ```bash
   make check-idf-version
   ```

2. If not installed, install ESP-IDF:
   ```bash
   make setup-idf
   ```

3. If already installed but needs update:
   ```bash
   make update-idf IDF_VERSION=v5.3.2
   ```

4. Verify the installation:
   ```bash
   make check-environment
   ```

## Configuration Options

The user can customize these variables:
- `IDF_PATH` - Installation directory (default: ~/repos/esp-idf)
- `IDF_VERSION` - ESP-IDF version (default: v5.3.2)
- `IDF_TARGETS` - Target chips (default: esp32,esp32s3,esp32c3)

Example with custom settings:
```bash
make setup-idf IDF_PATH=/custom/path IDF_VERSION=v5.4 IDF_TARGETS=esp32s3
```

## Post-Installation

After installation, remind the user to add the alias to their shell profile:
```bash
alias get_idf='. $HOME/repos/esp-idf/export.sh'
```

Report the installation status and any errors encountered.
