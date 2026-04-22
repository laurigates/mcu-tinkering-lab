# Containerized ESP-IDF Builds

## Architecture

All ESP-IDF projects build inside Docker containers using the shared `espressif/idf:v5.4` image. No local ESP-IDF installation is required.

- **Build/clean/menuconfig/shell** run inside the container via `docker compose run`
- **Flash/monitor** run natively on the host (USB passthrough on macOS is unreliable)
- Container engine is configurable: `CONTAINER_CMD=podman` for rootless builds

## Shared Configuration

### `tools/esp32.just` — Base variables and port detection

All ESP-IDF project justfiles import either `tools/esp32.just` (directly) or `tools/esp32-idf.just` (which imports `esp32.just` internally). Provides:

| Symbol | Purpose |
|--------|---------|
| `container_cmd` | `docker` or `podman` via `CONTAINER_CMD` env var |
| `compose_file` | Path to shared `docker-compose.yml` |
| `_detected_serial` | Auto-detected USB-serial adapter (`/dev/cu.usbserial-*`) |
| `_detected_s3` | Auto-detected ESP32-S3 USB-Serial-JTAG by VID `0x303a` |
| `_detected_uart` | Auto-detected CP2102/FTDI UART adapter |
| `_monitor_baud` | Serial monitor baud rate (default 115200, override via `MONITOR_BAUD` env var) |
| `require-port` | Private recipe — fails if `port` is empty |
| `_serial-monitor` | Private recipe — pyserial monitor using `port` and `_monitor_baud` |

### `tools/esp32-idf.just` — Shared build recipes

Standard ESP-IDF projects import `tools/esp32-idf.just` to get shared containerized recipes. This file imports `tools/esp32.just` internally, so only one import is needed.

| Symbol | Purpose |
|--------|---------|
| `_idf-build` | Private recipe — containerized `idf.py set-target + build` |
| `_idf-clean` | Private recipe — containerized `idf.py fullclean` |
| `_idf *args` | Private recipe — run any `idf.py` command in the container |
| `build` | Public — delegates to `_idf-build` (override to add pre-build steps) |
| `clean` | Public — delegates to `_idf-clean` |
| `menuconfig` | Public — containerized `idf.py menuconfig` |
| `shell` | Public — interactive container shell |
| `monitor` | Public — delegates to `_serial-monitor` |
| `flash-monitor` | Public — `flash` then `monitor` (requires project to define `flash`) |

### `tools/esphome.just` — Shared ESPHome recipes

ESPHome projects import `tools/esphome.just` for standard recipes (install, config, validate, compile, upload, wireless, logs, monitor, dashboard, clean, status, dev).

### Usage Patterns

```just
# Standard ESP-IDF project (gets build, clean, menuconfig, shell, monitor, flash-monitor):
import '../../../tools/esp32-idf.just'

project_dir := "packages/<domain>/my-project"
port := env("PORT", _detected_serial)  # USB-serial adapter boards
# or
port := env("PORT", _detected_s3)      # ESP32-S3 native USB boards
target := "esp32"                       # or "esp32s3"

# Override build to add pre-build steps (e.g., credentials):
[group: "build"]
build: credentials _idf-build

# Use _idf helper for additional idf.py commands:
[group: "build"]
size: (_idf "size")

# Non-building projects (orchestration, port detection only):
import '../../../tools/esp32.just'

# ESPHome projects:
import '../../../tools/esphome.just'
device_name := "my-device"
```

### Monitor Recipe

Projects using USB-serial adapters get the shared monitor automatically via `esp32-idf.just`.

ESP32-S3 projects with native USB-Serial-JTAG override `monitor` with their own scripts.

## Adding New ESP-IDF Projects

1. Create justfile with `import '../../../tools/esp32-idf.just'`
2. Set `project_dir`, `port`, and `target`
3. Define `flash` recipe (project-specific: binary name, chip, offsets)
4. Define `info` recipe (project-specific)
5. Override `build` if pre-build steps are needed (e.g., `build: credentials _idf-build`)
6. Use `require-port` as dependency for flash recipes
7. Register as a module in the root justfile: `mod name 'packages/<domain>/name'`

## Do Not

- Add `idf_path`, `check-idf`, or `source export.sh` patterns — those are the old local-install approach
- Hardcode `../../../docker-compose.yml` — use `{{compose_file}}` from the import
- Define `container_cmd`, `require-port`, `_monitor_baud`, or `_serial-monitor` locally — they come from the import
- Define `build`, `clean`, `menuconfig`, or `shell` with inline container commands — use `esp32-idf.just` shared recipes
- Copy the pyserial monitor block inline — use `_serial-monitor` instead
- Use absolute symlinks for vendored dependencies (`external/<name>`) — absolute targets like `/Users/...` don't resolve inside the container's `/workspace` bind mount. Use relative symlinks that stay under the repo root, e.g. `ln -sfn ../../../input-gaming/xbox-switch-bridge/external/bluepad32 external/bluepad32`.
