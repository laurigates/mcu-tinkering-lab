---
allowed-tools: Bash(git:*), Read, Grep, Glob
argument-hint: [files-or-commit]
description: Review embedded C/C++ code changes for best practices
---

## Context
Recent git changes:
!`git diff --stat HEAD~1`

## Task
Review the firmware code changes: $ARGUMENTS

If no specific files or commit provided, review the most recent changes.

## Review Checklist

### Memory Safety
- Check for buffer overflows and array bounds
- Verify proper null pointer checks
- Look for memory leaks (malloc without free)
- Check stack usage in functions

### Resource Management
- Verify all allocated resources are freed
- Check for proper GPIO initialization/deinitialization
- Look for semaphore/mutex deadlock potential
- Verify interrupt handlers are minimal

### ESP-IDF Best Practices
- Check task stack sizes are appropriate
- Verify FreeRTOS queue/semaphore usage
- Look for blocking calls in critical sections
- Check WiFi/network error handling

### Code Quality
- Verify meaningful variable/function names
- Check for magic numbers (should be #define or const)
- Look for duplicated code
- Verify error codes are properly propagated

### Security
- Check for hardcoded credentials (should be in Kconfig/NVS)
- Verify input validation
- Look for unsafe string functions (use snprintf vs sprintf)
- Check for proper certificate validation

Provide specific line numbers and suggestions for any issues found.
