---
name: esp32-debugging
description: Debug ESP32 firmware issues including compilation errors, runtime panics, memory issues, and communication failures
user-invocable: true
argument-hint: "[error-context]"
allowed-tools: Bash(just:*), Read, Grep, Glob
---

# ESP32 Firmware Debugging Guide

## When to Use This Skill

Apply this skill when the user:
- Encounters compilation errors in ESP-IDF projects
- Sees runtime panics or "Guru Meditation Error" messages
- Has memory-related crashes or stack overflows
- Experiences I2C/SPI/UART communication failures
- Needs help interpreting serial monitor output

`$ARGUMENTS` may contain error messages or context about the issue.

## Debugging Process

### Ask for Context First

If the error isn't clear from `$ARGUMENTS`, ask the user to provide:
1. Full error message or panic output
2. Which project they're building
3. Recent code changes

### 1. Compilation Error Analysis

**Run a fresh build to capture the error:**
```bash
just <project>::build 2>&1 | tail -100
```

**Missing Includes**
```
fatal error: driver/gpio.h: No such file or directory
```
Fix: Add the component to REQUIRES in `main/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "main.c"
    REQUIRES driver
)
```

**Undefined References**
```
undefined reference to 'some_function'
```
Fix: Ensure the component containing the function is in REQUIRES or PRIV_REQUIRES.

**Type Errors**
Look for mismatched types between function declarations and implementations.

### 2. Runtime Panic Analysis

**Guru Meditation Error Patterns**

| Error | Cause | Fix |
|-------|-------|-----|
| `StoreProhibited` | Writing to invalid memory | Check pointer initialization |
| `LoadProhibited` | Reading from invalid memory | Check null pointers |
| `InstrFetchProhibited` | Corrupted function pointer | Check callback assignments |
| `IntegerDivideByZero` | Division by zero | Add zero checks |

**Stack Overflow**
```
Guru Meditation Error: Core 0 panic'ed (Stack overflow)
```
Fix: Increase stack size in task creation:
```c
xTaskCreatePinnedToCore(task_fn, "name", 4096, NULL, 5, NULL, 0);
//                                        ^^^^ increase this
```

**Stack Smashing**
```
Stack smashing detected
```
Fix: Local buffer overflow — check array bounds and string operations.

### 3. Memory Debugging

**Check Heap Usage**
```c
ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
ESP_LOGI(TAG, "Min free heap: %lu", esp_get_minimum_free_heap_size());
```

**Common Memory Issues**
- Memory leak: Missing `free()` after `malloc()`
- Double free: Freeing same memory twice
- Use after free: Accessing freed memory

### 4. Communication Debugging

**I2C Issues**
```
E (1234) i2c: i2c_master_cmd_begin(xxx): I2C_NUM error
```
Checklist:
- Verify I2C address (7-bit vs 8-bit format)
- Check SDA/SCL GPIO pins
- Ensure pull-up resistors are present (4.7K typical)
- Verify clock frequency compatibility

**Serial/UART Issues**
- Baud rate mismatch
- TX/RX swapped
- Missing ground connection

**Dual-Controller Sync (I2C)**
- Check both controllers are running
- Verify I2C addresses match
- Check GPIO pin configuration

### 5. Build Commands for Debugging

```bash
# Clean build to eliminate stale objects
just <project>::clean && just <project>::build

# Start serial monitor
just <project>::monitor PORT=/dev/cu.usbserial-0001
```

### 6. Useful ESP-IDF Config Options

Enable in `sdkconfig.defaults` or via menuconfig:
- `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT` — Print panic info before reboot
- `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` — Detect stack overflow earlier
- `CONFIG_HEAP_POISONING_COMPREHENSIVE` — Detect heap corruption

## Common Fixes

| Symptom | Fix |
|---------|-----|
| Stack overflow | Increase task stack size in `xTaskCreate` |
| Memory leak | Check for missing `free()` calls |
| I2C timeout | Verify connections, pull-ups, addresses |
| Crash on startup | Increase `CONFIG_ESP_MAIN_TASK_STACK_SIZE` to 8192+ |
