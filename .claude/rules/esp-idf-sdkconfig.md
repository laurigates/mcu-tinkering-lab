# ESP-IDF sdkconfig Management

## How sdkconfig Works

ESP-IDF projects use a layered configuration system:

- `sdkconfig.defaults` — base settings checked into git
- `sdkconfig.<overlay>` — build-variant overrides (e.g., `sdkconfig.debug`)
- `sdkconfig` — **generated file**, lives in project root, NOT committed to git

`idf.py build` generates `sdkconfig` from defaults on first build. On subsequent builds, it **preserves existing values** and only adds new keys from defaults. This means changes to `sdkconfig.defaults` do NOT propagate to an existing `sdkconfig`.

## Staleness Rule

**After modifying any `sdkconfig.defaults` or `sdkconfig.*` overlay file, always delete the generated `sdkconfig` before building.**

```bash
rm sdkconfig
# then build (idf.py will regenerate from defaults + overlay)
```

Without this, the old values persist in the generated config, causing subtle bugs where code changes reference new config values that the build system silently ignores.

## When Modifying sdkconfig Files

1. Edit `sdkconfig.defaults` or the relevant overlay
2. Delete the generated `sdkconfig`
3. Run `idf.py fullclean` (or `just clean`) — the build directory caches config too
4. Rebuild

## sdkconfig Should Be Gitignored

The generated `sdkconfig` should be in `.gitignore`. Only `sdkconfig.defaults` and named overlays (`sdkconfig.debug`, etc.) belong in version control.

## Build Variant Pattern

Use `SDKCONFIG_DEFAULTS` env var to layer overlays:

```bash
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.debug" idf.py build
```

Later files override earlier ones. Each overlay should only contain the settings it needs to change.

## Stack Sizing

The default main task stack (`CONFIG_ESP_MAIN_TASK_STACK_SIZE=3584`) is often too small when initializing WiFi + BLE + USB. Set 8192 or higher for projects that initialize multiple subsystems in `app_main()`. Use `uxTaskGetStackHighWaterMark()` to verify headroom at runtime.

## Bluepad32 Requires UART as Primary Console

Bluepad32 v3.10.x (and its vendored btstack) unconditionally reference UART-console symbols in `uni_console.c` and `btstack_stdio_esp32.c` (`CONFIG_ESP_CONSOLE_UART_BAUDRATE`, `esp_console_dev_uart_config_t`, `esp_console_new_repl_uart`). ESP-IDF 5.4 only defines these when `CONFIG_ESP_CONSOLE_UART_DEFAULT` or `CONFIG_ESP_CONSOLE_UART_CUSTOM` is set, so `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` as primary breaks the build.

Keep UART as primary and select USB_SERIAL_JTAG as the secondary — ESP-IDF duplicates log output to both channels, so USB-C logging still works:

```
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y
```

Upstream issue: https://github.com/ricardoquesada/bluepad32/issues/209
