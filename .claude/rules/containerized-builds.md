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
| `_idf-build-checked bin` | Private — validated build: filters output to warnings/errors/size and fails fast if `build/{{bin}}.bin` is missing. Pass the app basename. |
| `_esp32-flash bin` | Private — native ESP32 flash (bootloader @0x1000, flash-size detect). Pass the app basename. |
| `_s3-flash bin` | Private — native ESP32-S3 flash (bootloader @0x0, 4MB @80MHz). Pass the app basename. |
| `_s3-reset` / `_s3-monitor` | Private — ESP32-S3 USB-Serial-JTAG reset / CDC-reset monitor (wrap the `tools/esp32s3-*.sh` scripts). |
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

# Shared flash/build/monitor recipes — pass bin_name (the compiled app basename)
# as a recipe ARGUMENT, not a shared variable (see note below):
bin_name := "robocar-main"            # = the CMake project() name, no .bin
build: (_idf-build-checked bin_name)  # validated build
flash: (_esp32-flash bin_name)        # or (_s3-flash bin_name) for esp32s3
reset: _s3-reset                      # esp32s3 only
monitor: _s3-monitor                  # esp32s3 (esp32 uses _serial-monitor)

# Non-building projects (orchestration, port detection only):
import '../../../tools/esp32.just'

# ESPHome projects:
import '../../../tools/esphome.just'
device_name := "my-device"
```

**Why `bin_name` is a recipe parameter, not a shared variable.** `just` treats an
import that *defaults* a variable a module also assigns as a conflict (`variable
`bin_name` has multiple definitions`), and an imported recipe that references
`{{bin_name}}` forces *every* importing module to define it at load time (or it
errors `variable not defined`). Passing it as a recipe argument
(`flash: (_s3-flash bin_name)`) sidesteps both and keeps the `.bin` name explicit
at the call site. Flash baud reads `${BAUD:-460800}` at the bash level, so no baud
variable is needed.

**Verify flash/build recipe changes with `just --dry-run` — CI does not exercise
them.** CI builds firmware via `esp-idf-ci-action` (not the justfile) and flash
runs host-native, so no automated gate covers these recipes; a wrong flash offset
bricks or fails-to-boot the device. Before changing a shared flash/build recipe,
confirm each consumer still expands to its original command:
`PORT=/dev/ttyDUMMY just <module>::flash --dry-run` (the `--dry-run` flag must
precede the recipe).

**Exotic flash layouts stay inline.** `_esp32-flash`/`_s3-flash` cover the two
standard single-app layouts only. Projects with a non-standard memory map (app
@0x12000, an extra `ota_data` segment, an esptool `@flash_args` argfile) or an
ESP32-CAM GPIO0 programming reminder keep their flash recipe inline so the offsets
stay explicit and auditable.

### Monitor Recipe

Projects using USB-serial adapters get the shared monitor automatically via `esp32-idf.just`.

ESP32-S3 projects with native USB-Serial-JTAG override `monitor` with their own scripts.

## Adding New ESP-IDF Projects

1. Create justfile with `import '../../../tools/esp32-idf.just'`
2. Set `project_dir`, `bin_name` (the compiled app basename), `port`, and `target`
3. Compose the shared flash/build recipes: `build: (_idf-build-checked bin_name)` and
   `flash: (_esp32-flash bin_name)` (or `(_s3-flash bin_name)`). Standard layouts only —
   keep an exotic flash recipe inline (see the flash-layout note above)
4. Define `info` recipe (project-specific)
5. Override `build` for pre-build steps if needed (e.g., `build: credentials (_idf-build-checked bin_name)`)
6. Use `require-port` as dependency for flash recipes (the shared `_*-flash` recipes already do)
7. Register as a module in the root justfile: `mod name 'packages/<domain>/name'`
8. Add a CI entry to `.github/project-matrix.json` (`system`, `project`, `path`, `target`,
   plus `fetch_bluepad32: true` if it vendors bluepad32) — the single `build.yml` matrix
   discovers it; no per-project `build-<project>.yml` is needed

## Do Not

- Add `idf_path`, `check-idf`, or `source export.sh` patterns — those are the old local-install approach
- Hardcode `../../../docker-compose.yml` — use `{{compose_file}}` from the import
- Define `container_cmd`, `require-port`, `_monitor_baud`, or `_serial-monitor` locally — they come from the import
- Define `build`, `clean`, `menuconfig`, or `shell` with inline container commands — use `esp32-idf.just` shared recipes
- Copy the pyserial monitor block inline — use `_serial-monitor` instead
- Inline the native `esptool` flash block or the S3 reset/monitor scripts for a
  standard-layout project — compose the shared `_esp32-flash` / `_s3-flash` /
  `_s3-reset` / `_s3-monitor` recipes instead (exotic layouts excepted)
- Create a per-project `build-<project>.yml` CI workflow — add a
  `.github/project-matrix.json` entry instead; the single `build.yml` matrix builds it
- Use absolute symlinks for vendored dependencies (`external/<name>`) — absolute targets like `/Users/...` don't resolve inside the container's `/workspace` bind mount. Use relative symlinks that stay under the repo root, e.g. `ln -sfn ../../../input-gaming/xbox-switch-bridge/external/bluepad32 external/bluepad32`.
