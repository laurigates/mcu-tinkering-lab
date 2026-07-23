# ESP_LOG Format Argument Must Be a String Literal

## The Rule

The `ESP_LOGx` macros (`ESP_LOGE/W/I/D/V`) require their **format argument to
be a compile-time string literal**. The logging macros string-concatenate the
format with color/prefix literals at expansion time (`LOG_FORMAT(letter,
format)` token-pastes `"\033[…m" format "\033[0m"`). C only concatenates
adjacent *string literals*, so anything that is not a literal — a ternary, a
variable, a function call — breaks the expansion.

## The Failure

```c
// WRONG — ternary is not a string literal
ESP_LOGI(TAG, connected ? "Host connected" : "Host disconnected");
```

Compiles to a syntax error far from the obvious cause:

```
error: expected ')' before 'connected'
```

The error points at the log macro's internals (`esp_log_color.h`), not at an
obviously malformed call, so it reads as a toolchain problem rather than a
misused macro.

## The Fix

Keep the format a literal; pass the dynamic part as a `%s` (or other) argument:

```c
// RIGHT — literal format, ternary passed as an argument
ESP_LOGI(TAG, "Host %s", connected ? "connected" : "disconnected");
```

Same rule for any non-literal: build the message with format specifiers and
pass the values as arguments — never `ESP_LOGI(TAG, some_char_ptr)` or
`ESP_LOGI(TAG, cond ? a : b)`.

## When It Bites

- Logging a state change with a ternary describing the new state.
- Passing a `const char *` variable directly as the format (also a
  format-string-injection risk if the string is attacker-influenced).
- Any project under `packages/` — this is an ESP-IDF-wide gotcha, not specific
  to one firmware.

First hit 2026-07 in `facedancer-espdancer-fw/main/main.c` (PR #415).
