# esp32s3-gemini-vision

XIAO ESP32-S3 Sense + **Gemini Robotics-ER 1.5** object detection, served through a browser UI at `http://xiao-vision.local/`.

- MJPEG stream from the onboard OV2640 (`/stream`)
- Single-tap **Detect now** button → Gemini returns labeled bounding boxes
- Optional **Auto mode** (3–30 s interval)
- Boxes are overlaid on the live view in a canvas; coordinates are Gemini's native 0–1000 normalized `[ymin, xmin, ymax, xmax]` rescaled to the viewport

## Why Gemini Robotics-ER?

It is tuned for spatial reasoning (grounded boxes + points) and is priced like Flash — roughly **$0.0006 per frame** at default settings (~360 input + 200 output tokens). The free tier is sufficient for casual use.

## Hardware

- Seeed Studio **XIAO ESP32-S3 Sense** (ESP32-S3 + OV2640 + 8 MB octal PSRAM, native USB-C with USB-Serial-JTAG)
- No additional wiring — the camera is internal to the Sense expansion board

## Setup

```bash
# 1. Fill in credentials
cp main/credentials.h.example main/credentials.h
$EDITOR main/credentials.h      # WIFI_SSID, WIFI_PASSWORD, GEMINI_API_KEY

# 2. Get a Gemini API key
#    https://aistudio.google.com/apikey

# 3. Build (containerized — no local ESP-IDF needed)
just build

# 4. Flash + monitor
just flash-monitor
```

The serial log reports the assigned IP; open `http://xiao-vision.local/` (or the IP) in a browser on the same LAN.

## HTTP API

| Route | Method | Description |
|---|---|---|
| `/` | GET | Web UI |
| `/stream` | GET | MJPEG (`multipart/x-mixed-replace`) — served on port **:81** (a second httpd instance so the never-ending stream loop doesn't starve the JSON endpoints on :80) |
| `/snapshot` | GET | Single JPEG |
| `/detect` | GET/POST | Enqueue a Gemini detection (202) |
| `/metadata` | GET | Latest detection JSON `{timestamp_ms, latency_ms, busy, objects:[...]}` |
| `/config?interval=5` | GET | Set auto-mode interval in seconds (0 = off) |

`objects` is an array of `{"label": "...", "box_2d": [ymin, xmin, ymax, xmax]}` with integer coordinates 0–1000.

## Cost

| Cadence | Approx. cost (paid tier) |
|---|---|
| On-demand (1 click) | ~$0.0006 |
| Auto @ 5 s | ~$0.43 / hr |
| Auto @ 30 s | ~$0.07 / hr |

Token usage is logged each request (`tokens: prompt=… output=… total=…`) for validation.

## Tuning

- `FRAMESIZE_VGA` (`main/main.c`) — 640×480 by default; drop to `FRAMESIZE_QVGA` for faster inference, or `FRAMESIZE_SVGA` for more detail.
- `GEMINI_THINKING_BUDGET` (`main/gemini_client.c`) — 0 minimises latency; raise (e.g. 512) for higher-accuracy reasoning.
- `jpeg_quality = 12` — lower = higher quality / larger payload.

## Files

```
main/
├── main.c               # app_main, camera, WiFi, mDNS, HTTP handlers, worker task
├── gemini_client.c/.h   # HTTPS POST to generativelanguage.googleapis.com
├── base64.c/.h          # JPEG → base64
├── index.html           # Single-page UI (embedded via EMBED_TXTFILES)
├── credentials.h.example
└── credentials.h        # (gitignored)
```
