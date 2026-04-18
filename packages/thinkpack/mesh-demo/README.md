# ThinkPack Mesh Demo

Validates the **Phase 1 foundation** of the ThinkPack modular toy system on two
ESP32-S3 boards. Exercises peer discovery, leader election, and the event
callback path — no peripherals required.

See [issue #194](https://github.com/laurigates/mcu-tinkering-lab/issues/194)
for the full Phase 1 specification.

## Hardware

Any two ESP32-S3 boards — SuperMini, DevKitC, or similar. No wiring needed
beyond a USB cable per board for power and serial monitoring.

## Identity selection

Identity is derived automatically from the last byte of each board's burned-in
WiFi-STA MAC address:

| Last MAC byte | Box type   | Capabilities                        |
|---------------|------------|-------------------------------------|
| even          | GLOWBUG    | `CAP_LED_RING \| CAP_IMU \| CAP_LIGHT_SENSE` |
| odd           | BOOMBOX    | `CAP_AUDIO_OUT \| CAP_POTS \| CAP_RHYTHM`   |

No flashing different binaries per board — just plug in two boards and both
will auto-differentiate.

## Quick start

```bash
# Board A
just build
PORT=/dev/cu.usbmodem1 just flash
PORT=/dev/cu.usbmodem1 just monitor

# Board B (separate terminal)
PORT=/dev/cu.usbmodem2 just flash
PORT=/dev/cu.usbmodem2 just monitor
```

## Expected output

Within **1 second** of both boards running:

```
I (350) demo: peer discovered: aa:bb:cc:dd:ee:01
```

Within **3 seconds**, leader election resolves:

```
I (2100) demo: leader elected: aa:bb:cc:dd:ee:01
I (2100) demo: I AM THE LEADER          # on the winning board
I (2100) demo: following aa:bb:cc:dd:ee:01  # on the other board
```

Every 10 seconds both boards log current peer count:

```
I (10350) demo: peer_count = 1
```

If a board is reset, `peer lost` appears after the beacon timeout and a new
election runs automatically.

## Troubleshooting

**No peers seen after 10 s:**
- Verify both boards are running the same build (channel 1, 500 ms beacons).
- Check USB cables supply power (not charge-only cables).
- Confirm boards share the same 2.4 GHz environment — ESP-NOW range is ~200 m
  line-of-sight; walls reduce this but a desk apart is always fine.

**Build fails with "component not found":**
- Ensure you are building from the repo root via `just thinkpack-mesh-demo::build`.
  The `EXTRA_COMPONENT_DIRS` path in `CMakeLists.txt` resolves relative to the
  project root inside the container.
