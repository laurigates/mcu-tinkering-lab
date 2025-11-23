---
description: Verify development environment is properly configured
allowed-tools: Bash(make:*), Bash(ls:*)
---

# Check Development Environment

Verify that the MCU development environment is properly configured.

## Run Environment Check

Execute the environment verification:

```bash
make check-environment
```

## Additional Checks

Also check:

1. ESP-IDF version:
   ```bash
   make check-idf-version
   ```

2. Available serial ports:
   ```bash
   ls -la /dev/cu.usbserial-* /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
   ```

3. Project structure:
   ```bash
   make list-projects
   ```

## Summarize Results

Provide a clear summary:

- ESP-IDF status (installed/missing, version)
- Serial ports available
- Projects found
- Any missing dependencies

## Recommendations

If issues found:
- Missing ESP-IDF: "Run 'make setup-idf' to install"
- No serial ports: "Connect your ESP32 device via USB"
- Missing tools: "Run 'make install-dev-tools' to install development tools"
