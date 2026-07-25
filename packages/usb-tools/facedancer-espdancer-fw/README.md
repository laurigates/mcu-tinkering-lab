# facedancer-espdancer-fw

ESP32-S3 firmware that turns an [ESP32-S3](https://www.espressif.com/en/products/socs/esp32s3)
into a **raw USB relay** for the [facedancer](https://github.com/greatscottgadgets/facedancer)
`espdancer` backend. The S3 enumerates as **whatever USB device the
controlling PC tells it to** — the host supplies the full descriptor set and
endpoint topology; the firmware just relays USB primitives over a WiFi control
channel.

> **Status: scaffold (Milestone 0).** The firmware boots, advertises the
> `espdancer-log` WiFi SoftAP, and answers `HELLO` / `GET_VERSION` on TCP
> 4444. **No USB emulation yet.** See [`CHECKLIST.md`](CHECKLIST.md) for the
> milestone backlog; [`components/raw_usb`](components/) (Milestone 1) is the
> hard part.

## How it works (target)

```
┌──────────────┐   WiFi SoftAP    ┌──────────────┐    USB-C     ┌──────────────┐
│ Controlling  │ ────────────────▶│  ESP32-S3    │─────────────▶│   Target     │
│   PC running │   TCP 4444       │  espdancer   │  Full-Speed │   host       │
│ espdancer.py │   framed TLV     │  raw relay   │  enumerated │ (sees the    │
└──────────────┘                  └──────────────┘             └──────────────┘
                                                                emulated device)
```

The firmware holds no USB device-class knowledge — it's a dumb raw-DCD relay.
Swap the emulated device by sending a new `CONNECT` frame, no reflash.

## Hardware required

- **Waveshare ESP32-S3-Zero** (recommended — 4 MB flash, 2 MB PSRAM, WS2812 on
  GPIO21, native USB-OTG) or any ESP32-S3 with native USB-OTG.
- **USB-C cable** from the S3 to the target host.
- **(debug-uart only)** a CP2102/CH340 USB-UART adapter on GPIO43 (TX) / GPIO44 (RX).
- **(production/debug-usb)** any device that can join the `espdancer-log` WiFi.

> **Note:** a standard ESP32 (WROOM/WROVER) will **not** work — no native USB.

## Building

Requires ESP-IDF v5.4+ (builds run in Docker via the shared `docker-compose.yml`).

```bash
just build              # production
just build-debug-uart   # TinyUSB ON + UART logging (CP2102 on GPIO43/44)
just build-debug-jtag   # TinyUSB OFF + USB-Serial-JTAG logging (no USB emu)
just build-debug-usb    # TinyUSB ON + verbose WiFi UDP logging
just flash              # flash to the auto-detected S3 port
just log-listen         # UDP log listener (join 'espdancer-log' WiFi first)
```

Or without `just`:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash
```

**Switching variants:** delete the generated `sdkconfig` cache first:
`rm sdkconfig && just build-debug-uart`.

## Using the control channel (Milestone 0)

1. Flash the firmware.
2. Join the `espdancer-log` WiFi network (password `espdancer`).
3. Open a TCP connection to `192.168.4.1:4444`.
4. Send a `HELLO` frame and read the `HELLO_REPLY`:

   ```sh
   # magic(2) + len_be(3) + msg_id(0x01) + proto(0x0001) + crc8
   printf '\xf0\xd0\x00\x03\x01\x00\x01' | nc 192.168.4.1 4444 | xxd
   ```

See [`docs/rpc-protocol.md`](docs/rpc-protocol.md) for the full wire contract.

## Limits

- **Full Speed only** (12 Mbps) — internal OTG-FS PHY. High Speed needs an
  external ULPI PHY that no common devkit routes.
- **No USBProxy / Meddler-In-The-Middle** — the single OTG port can't be both
  USB host and USB device at once. Same limitation MAX3421 Facedancer boards
  carry.
- **Endpoint count** bounded by the OTG-FS block (~6–8 on the S3). Oversized
  descriptors fail with a clear error from the Python backend.

## Architecture

```
main/main.c                  # entry: LED + WiFi AP + rpc task + idle
components/
├── status_led/              # WS2812 RMT (non-DMA) — emulator-state indicator
├── log_udp/                 # SoftAP "espdancer-log" + UDP log broadcast
└── usb_rpc/                 # TCP server + framed-TLV dispatcher (host iface)
docs/
├── adrs/                    # ADRs 0001–0005
└── rpc-protocol.md          # host↔device wire contract
CHECKLIST.md                 # working backlog (start here)
```

## Related projects

- [facedancer](https://github.com/greatscottgadgets/facedancer) — the Python
  USB-emulation library this firmware serves; the `espdancer.py` backend lands
  there (Milestone 3).
- [xbox-switch-bridge](../input-gaming/xbox-switch-bridge) — provenance;
  `sdkconfig.defaults`, the build-variant system, `status_led`, `log_udp`,
  and the single-writer pattern were lifted from it.
