# CLAUDE.md — esp32-cam-webserver

Live MJPEG video streaming web server for the ESP32-CAM (AI-Thinker) module. Part of the [mcu-tinkering-lab](https://github.com/laurigates/mcu-tinkering-lab) monorepo.

## Tech Stack

| Layer | Technology |
|-------|-----------|
| MCU | ESP32 (AI-Thinker ESP32-CAM) |
| Camera | OV2640 via `espressif/esp32-camera` component |
| Framework | ESP-IDF v5.4+ |
| Build | CMake (via ESP-IDF), containerized via Docker |
| Task runner | just |
| Formatter | clang-format (Google style, 4-space indent) |
| Linter | cppcheck |

## Development Workflow

All builds run inside Docker containers — no local ESP-IDF installation required. Flash and monitor run natively on the host.

### Getting Started

```bash
# Set up WiFi credentials (copy template and edit)
just credentials
$EDITOR main/credentials.h

# Build firmware (containerized)
just build

# Flash to device (GPIO0 must be connected to GND first)
just flash

# Monitor serial output
just monitor

# Full cycle: build → flash → monitor
just develop
```

Run `just --list` to see all available recipes.

### Flashing ESP32-CAM

The AI-Thinker ESP32-CAM requires GPIO0 pulled low to enter bootloader mode:

1. Connect GPIO0 to GND on the board
2. Power on or reset the board
3. Run `just flash`
4. **Disconnect GPIO0 from GND** after flashing completes
5. Reset the board to run the firmware

### Code Quality

```bash
just lint          # cppcheck on main/
just format        # clang-format -i (modifies files)
just format-check  # clang-format --dry-run --Werror (CI-safe)
```

## Architecture

### HTTP Endpoints

| Endpoint | Handler | Description |
|----------|---------|-------------|
| `GET /` | `root_handler` | HTML page with embedded `<img src="/stream">` |
| `GET /stream` | `stream_handler` | MJPEG stream (`multipart/x-mixed-replace`) |

### Startup Sequence

`app_main()` in `main/main.c`:
1. Initialize NVS flash
2. Initialize OV2640 camera (AI-Thinker pin mapping hardcoded)
3. **1-second delay** — prevents brownout from simultaneous camera + WiFi power draw
4. Connect WiFi (station mode, up to 5 retries)
5. Start HTTP server on port 80

### Camera Configuration

- Resolution: SVGA (800×600), JPEG format, quality 15
- Frame buffer count: 2 (double-buffered)
- Target ~24 fps (`vTaskDelay(42ms)` in stream loop)
- Pins defined in `main.c` for AI-Thinker board; `sdkconfig.defaults` mirrors these for the camera component

### Power Considerations

Brownout detector is set to level 5 (2.70 V) and WiFi TX power is capped at 15 dBm. These settings in `sdkconfig.defaults` prevent the board from resetting during the high-current WiFi association phase.

## Conventions

### Credentials

- Copy `main/credentials.h.example` → `main/credentials.h` and fill in WiFi SSID/password
- `credentials.h` is gitignored — **never commit it**
- `just credentials` automates the copy step

### sdkconfig

- `sdkconfig.defaults` is committed; generated `sdkconfig` is gitignored
- After editing `sdkconfig.defaults`, delete `sdkconfig` and run `just clean` before rebuilding
- See monorepo rule: `.claude/rules/esp-idf-sdkconfig.md`

### Code Style

- Google C style, 4-space indent, 100-char line limit (`.clang-format` at repo root)
- Pre-commit hooks enforce formatting; run `pre-commit run --all-files` to check locally

### Commit Messages

Conventional commits: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `ci:`, `build:`

## Project Structure

```
esp32-cam-webserver/
├── main/
│   ├── main.c                 # App entry: camera init, WiFi, HTTP server
│   ├── credentials.h.example  # WiFi credentials template
│   ├── credentials.h          # Your credentials (gitignored)
│   ├── CMakeLists.txt
│   └── idf_component.yml      # esp32-camera component dependency
├── CMakeLists.txt
├── sdkconfig.defaults          # Committed ESP-IDF config baseline
├── justfile                    # Build/flash/monitor recipes
└── README.md
```

## Related Resources

- Monorepo root CLAUDE.md: `../../../CLAUDE.md`
- Shared justfile helpers: `../../../tools/esp32.just`
- Docker service definition: `../../../docker-compose.yml`
- ESP-IDF sdkconfig rules: `../../../.claude/rules/esp-idf-sdkconfig.md`
- Containerized build rules: `../../../.claude/rules/containerized-builds.md`
