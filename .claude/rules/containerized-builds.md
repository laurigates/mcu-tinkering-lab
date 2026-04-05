# Containerized ESP-IDF Builds

## Architecture

All ESP-IDF projects build inside Docker containers using the shared `espressif/idf:v5.4` image. No local ESP-IDF installation is required.

- **Build/clean/menuconfig/shell** run inside the container via `docker compose run`
- **Flash/monitor** run natively on the host (USB passthrough on macOS is unreliable)
- Container engine is configurable: `CONTAINER_CMD=podman` for rootless builds

## Shared Configuration: `tools/esp32.just`

All ESP-IDF project justfiles import `tools/esp32.just` which provides:

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

### Usage Pattern

```just
import '../../../tools/esp32.just'

port := env("PORT", _detected_serial)  # USB-serial adapter boards
# or
port := env("PORT", _detected_s3)      # ESP32-S3 native USB boards
```

### Monitor Recipe

Projects using USB-serial adapters delegate to the shared monitor:

```just
[group: "device"]
monitor: _serial-monitor
```

ESP32-S3 projects with native USB-Serial-JTAG use their own monitor scripts instead.

## Adding New ESP-IDF Projects

1. Create justfile with `import '../../../tools/esp32.just'`
2. Set `project_dir`, `port`, and `target`
3. Use `{{container_cmd}} compose -f {{compose_file}} run --rm -w /workspace/{{project_dir}} esp-idf` for build recipes
4. Use `require-port` as dependency for flash recipes
5. Use `_serial-monitor` as dependency for monitor recipes (or write custom if needed)
6. Register as a module in the root justfile: `mod name 'packages/esp32-projects/name'`

## Do Not

- Add `idf_path`, `check-idf`, or `source export.sh` patterns — those are the old local-install approach
- Hardcode `../../../docker-compose.yml` — use `{{compose_file}}` from the import
- Define `container_cmd`, `require-port`, `_monitor_baud`, or `_serial-monitor` locally — they come from the import
- Copy the pyserial monitor block inline — use `_serial-monitor` instead
