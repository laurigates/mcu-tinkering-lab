---
name: check-env
description: Verify development environment is properly configured
allowed-tools: Bash(just:*), Bash(ls:*)
---

# Check Development Environment

Verify that the MCU development environment is properly configured.

## Run Environment Check

Execute the environment verification:

```bash
just check-environment
```

## Additional Checks

Also check:

1. Available serial ports:
   ```bash
   ls -la /dev/cu.usbserial-* /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
   ```

2. Docker availability:
   ```bash
   docker --version
   ```

3. Project structure:
   ```bash
   just --list
   ```

## Summarize Results

Provide a clear summary:

- Docker status (installed/missing, version)
- Serial ports available
- Projects found
- Any missing dependencies

## Recommendations

If issues found:
- Missing Docker: "Install Docker to build ESP-IDF projects"
- No serial ports: "Connect your ESP32 device via USB"
- Missing tools: "Run `just install-dev-tools` to install development tools"
