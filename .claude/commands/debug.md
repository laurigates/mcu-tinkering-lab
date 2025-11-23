---
description: Analyze and debug ESP32 compilation or runtime errors
argument-hint: "[error-context]"
allowed-tools: Bash(make:*), Read, Grep
---

# Debug ESP32 Issues

Analyze and help debug ESP32 compilation errors, runtime panics, or other issues.

## Context

$ARGUMENTS may contain error messages or context about the issue.

## Debugging Steps

### 1. For Compilation Errors

If the user mentions build/compilation errors:

1. Run a fresh build to capture the error:
   ```bash
   make robocar-build-main 2>&1 | tail -100
   ```

2. Analyze the error output for:
   - Missing header files -> Check CMakeLists.txt REQUIRES
   - Undefined references -> Check component registration
   - Type errors -> Check struct/function definitions

### 2. For Runtime Panics

If the user mentions crashes or panics:

1. Look for panic patterns in their description:
   - "Guru Meditation Error" -> Stack overflow or invalid memory access
   - "Stack smashing" -> Buffer overflow
   - "LoadProhibited" -> Null pointer dereference

2. Suggest running monitor to capture the panic:
   ```bash
   make robocar-monitor-main PORT=/dev/xxx
   ```

### 3. For I2C/Communication Errors

If dual-controller sync issues:
- Check both controllers are running
- Verify I2C addresses match
- Check GPIO pin configuration

## Common Fixes

Provide specific fixes based on the error type:

- **Stack overflow**: Increase task stack size in xTaskCreate
- **Memory leak**: Check for missing free() calls
- **I2C timeout**: Verify connections, pull-ups, addresses

## Ask for More Context

If the error isn't clear, ask the user to provide:
1. Full error message or panic output
2. Which project they're building
3. Recent code changes
