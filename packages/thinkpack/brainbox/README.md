# ThinkPack Brainbox

The mesh coordinator for the ThinkPack modular toy system. Runs on an **ESP32-S3**
(WROOM N8R8 recommended for PSRAM), connects to WiFi, and uses an LLM to generate
collective play directives for the surrounding peer boxes (Glowbug, Boombox, etc.)
over ESP-NOW.

See [issue #196](https://github.com/laurigates/mcu-tinkering-lab/issues/196) for
the full Phase 3B requirements.

---

## Hardware

| Component | Value |
|-----------|-------|
| MCU | ESP32-S3 WROOM N8R8 (8 MB flash, 8 MB OPI PSRAM) |
| Status LED | WS2812 single pixel — GPIO 4 |
| Piezo buzzer | Passive piezo — GPIO 5 |
| Button | Tactile switch — GPIO 9 (pull-up, active LOW) |
| OLED display | SSD1306 I2C — TBD (driver stubbed; serial log only in v0.1) |

---

## Pinout

See [WIRING.md](WIRING.md) for a detailed pinout table and ASCII wiring diagram.

---

## Quick Start

### 1. Credentials

```bash
cp main/credentials.h.example main/credentials.h
$EDITOR main/credentials.h
```

Fill in `WIFI_SSID`, `WIFI_PASSWORD`, and the key for your chosen AI backend
(`CLAUDE_API_KEY` or `GEMINI_API_KEY`; Ollama needs no key).

### 2. Select AI backend

```bash
just menuconfig
# Navigate to: ThinkPack Brainbox Configuration → AI Backend Selection
# Choose: Claude API / Ollama / Gemini
# Save and exit
```

Default is **Ollama** (no API key required; auto-discovered via mDNS).

### 3. Build and flash

```bash
just build
just flash-monitor
```

Override the port if auto-detection fails:

```bash
PORT=/dev/ttyACM1 just flash-monitor
```

---

## AI Backend Selection

| Backend | Kconfig symbol | Key required | Discovery |
|---------|---------------|--------------|-----------|
| Ollama | `AI_BACKEND_OLLAMA` (default) | No | mDNS (`_ollama._tcp.local`) |
| Claude API | `AI_BACKEND_CLAUDE` | `CLAUDE_API_KEY` | Fixed URL |
| Gemini | `AI_BACKEND_GEMINI` | `GEMINI_API_KEY` | Fixed URL |

---

## Expected Behaviour

1. On boot the status LED lights red and WiFi connects.
2. Once peers are discovered via ESP-NOW beacons, the Brainbox wins leader
   election (highest CAP score: CAP_LLM + CAP_WIFI + CAP_DISPLAY).
3. On each peer discovery event or button press, the Brainbox queries the LLM
   with a prompt describing the current group, parses the JSON response, and
   sends command packets to each peer.
4. Every 5 minutes a "periodic surprise" query fires if peers are present (FR-T20).
5. The status LED cycles through red → amber → green → blue → violet on each
   button press; a 200 ms 880 Hz chime plays simultaneously.

---

## Troubleshooting

**Build fails — `main/credentials.h` not found**
Run `cp main/credentials.h.example main/credentials.h` and fill in credentials.

**AI queries return errors**
- Claude/Gemini: verify the API key in `credentials.h` and that WiFi is connected.
- Ollama: ensure an Ollama server is reachable on the local network. The brainbox
  discovers it via mDNS (`_ollama._tcp.local`). Check `CONFIG_MDNS_ENABLED=y` in
  `sdkconfig.defaults`.

**No peers appearing**
Confirm peer boxes are on the same WiFi channel (channel 1 by default). Check that
`CONFIG_ESP_WIFI_ESPNOW_ENABLED=y`.

**OLED shows nothing**
The SSD1306 driver is stubbed in v0.1. Status output appears on the serial monitor
at 115200 baud. OLED rendering is tracked as a follow-up in issue #196.

**Stack overflow in `brain_status` task**
Increase `CONFIG_ESP_MAIN_TASK_STACK_SIZE` in `sdkconfig.defaults` and rebuild.

---

## Project Structure

```
packages/thinkpack/brainbox/
├── main/
│   ├── main.c                  # app_main wiring
│   ├── group_mode.{c,h}        # LLM trigger queue + worker task
│   ├── response_parser.{c,h}   # JSON → command payloads
│   ├── display_manager.{c,h}   # OLED stub (serial log)
│   ├── standalone_mode.{c,h}   # Button + piezo + WS2812
│   ├── thinkpack_ai.{c,h}      # Backend vtable selector
│   ├── prompt_builder.{c,h}    # LLM prompt construction
│   ├── wifi_manager.{c,h}      # WiFi STA + reconnection
│   ├── credentials_loader.{c,h}# NVS → env → credentials.h
│   ├── ollama_discovery.{c,h}  # mDNS Ollama discovery
│   ├── *_text_backend.c        # Claude / Gemini / Ollama implementations
│   ├── Kconfig.projbuild       # AI_BACKEND_* Kconfig choice
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # led_strip managed component
│   └── credentials.h.example   # Template — copy and fill in
├── CMakeLists.txt              # Project root; adds ../../components
├── sdkconfig.defaults          # ESP32-S3 + PSRAM + HTTPS + mDNS defaults
├── version.txt                 # 0.1.0
├── justfile                    # Build recipes
├── README.md
└── WIRING.md
```
