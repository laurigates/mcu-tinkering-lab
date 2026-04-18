# Nintendo Switch Pro Controller USB Protocol Reference

Reference for emulating a wired Switch Pro Controller over USB HID.
Based on [dekuNukem/Nintendo_Switch_Reverse_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering).

## USB Handshake Sequence (Phase 1)

The Switch sends `0x80` HID output reports; the controller responds with `0x81` input reports.

| Command | Payload | Reply | Purpose |
|---------|---------|-------|---------|
| `80 01` | — | `81 01 00 02` + MAC (6 bytes, reversed) | Connection status + MAC + controller type |
| `80 02` | — | `81 02` | UART handshake to BT chip. **Once per session only.** |
| `80 03` | — | `81 03` | Switch UART to 3 Mbit. Requires `80 02` first. |
| `80 04` | — | (none) | Force USB-only mode. Marks handshake complete. |
| `80 05` | — | (none) | Disable USB timeout / re-enable BT timeout |

After `80 04`, the Switch transitions to the subcommand protocol.

## Subcommand Protocol (Phase 2)

### Output Report 0x01 (Switch → Controller)

```
Byte  0:    Report ID (0x01)
Byte  1:    Packet counter (0x0–0xF, increments per packet)
Bytes 2–9:  Rumble data (8 bytes; neutral = 00 01 40 40 00 01 40 40)
Byte  10:   Subcommand ID
Bytes 11+:  Subcommand arguments
```

### Input Report 0x21 (Controller → Switch, subcommand reply)

```
Byte  0:    Timer (0x00–0xFF, fast increment)
Byte  1:    Battery level (high nibble 0–9) | connection info (low nibble)
Bytes 2–4:  Button status (right, shared, left)
Bytes 5–7:  Left stick (12-bit packed)
Bytes 8–10: Right stick (12-bit packed)
Byte  11:   Vibrator input report
Byte  12:   ACK byte (MSB=1 = success)
Byte  13:   Echo of subcommand ID
Bytes 14+:  Subcommand reply data (max 35 bytes)
```

### ACK Byte Patterns (Byte 12)

| Prefix | Meaning |
|--------|---------|
| `0x80` | Standard ACK |
| `0x90` | SPI flash read ACK |
| `0xA0` | NFC/IR MCU config ACK |
| `0xB0` | Player lights query |
| `0xC0` | IMU register read |
| `0xD0` | Voltage query |

Unknown/unhandled subcommands: reply with `0x80` + subcmd_id, data byte `0x03`.

## Typical Setup Sequence

The Switch sends these subcommands in order after the USB handshake:

1. `0x02` — Request device info
2. `0x08 0x00` — Reset shipment low-power state
3. `0x10` — SPI flash reads (multiple, see below)
4. `0x04` — Trigger buttons elapsed time
5. `0x03 0x30` — Set input report mode to standard full (60 Hz)
6. `0x40 0x01` — Enable IMU
7. `0x41` — Set IMU sensitivity
8. `0x48 0x01` — Enable vibration
9. `0x30` — Set player lights
10. `0x38` — Set HOME button LED

After setup, the Switch expects continuous `0x30` input reports.

**Critical**: Do NOT send `0x30` input reports during the setup phase — they share the HID IN endpoint with `0x21` subcommand replies and will starve the replies.

## Subcommand Reference

### 0x02 — Request Device Info

Reply data (bytes 14+):

```
Byte  0–1: Firmware version (e.g., 0x04 0x33)
Byte  2:   Device type (0x03 = Pro Controller)
Byte  3:   Unknown (0x02)
Bytes 4–9: MAC address (big-endian)
Byte  10:  Unknown (0x01)
Byte  11:  Color source (0x01 = use firmware defaults, 0x02 = use SPI colors)
```

### 0x03 — Set Input Report Mode

| Argument | Mode |
|----------|------|
| `0x30` | Standard full mode (continuous 60 Hz) |
| `0x3F` | Simple HID mode (button-triggered) |

### 0x04 — Trigger Buttons Elapsed Time

Reply: 7× uint16LE values in 10ms units (return zeros if not tracking).

### 0x08 — Set Shipment Low Power State

Argument `0x00` = disable low power, `0x01` = enable.

### 0x10 — SPI Flash Read

Request (bytes 11+): 4-byte LE address + 1-byte size (max 0x1D = 29 bytes).

Reply (bytes 14+): address echo (4 bytes) + size echo (1 byte) + data.

ACK byte must be `0x90` (not `0x80`).

### 0x30 — Set Player Lights

Argument: bitfield — lower 4 bits = steady LEDs, upper 4 bits = flashing LEDs.

### 0x38 — Set HOME Button LED

25-byte configuration (can ACK and ignore for emulation).

### 0x40 — Enable IMU

`0x00` = off, `0x01` = on.

### 0x41 — Set IMU Sensitivity

4 bytes: gyro range, accel range, gyro performance rate, accel anti-aliasing filter bandwidth.

### 0x48 — Enable Vibration

`0x00` = off, `0x01` = on.

## Standard Input Report 0x30

Sent continuously after setup completes. The `0x03 0x30` subcommand requests standard full mode (software rate target); actual USB polling is at 125 Hz (8ms endpoint interval).

```
Byte  0:     Timer (0x00–0xFF)
Byte  1:     Battery (high nibble) | connection info (low nibble)
Byte  2:     Right buttons
Byte  3:     Shared buttons
Byte  4:     Left buttons
Bytes 5–7:   Left stick (12-bit packed)
Bytes 8–10:  Right stick (12-bit packed)
Byte  11:    Vibrator input report
Bytes 12–47: IMU data (3 frames × 12 bytes; zero-fill if IMU not implemented)
```

Total: 48 bytes (after report ID).

### Button Byte Layout

**Byte 2 — Right buttons**:

| Bit | Button |
|-----|--------|
| 0 | Y |
| 1 | X |
| 2 | B |
| 3 | A |
| 6 | R |
| 7 | ZR |

**Byte 3 — Shared buttons**:

| Bit | Button |
|-----|--------|
| 0 | Minus |
| 1 | Plus |
| 2 | Right Stick click |
| 3 | Left Stick click |
| 4 | Home |
| 5 | Capture |
| 7 | Charging Grip |

**Byte 4 — Left buttons**:

| Bit | Button |
|-----|--------|
| 0 | Down |
| 1 | Up |
| 2 | Right |
| 3 | Left |
| 6 | L |
| 7 | ZL |

### 12-Bit Stick Packing

```c
// Pack: 12-bit X and Y into 3 bytes
bytes[0] = x & 0xFF;
bytes[1] = ((y & 0x0F) << 4) | ((x >> 8) & 0x0F);
bytes[2] = (y >> 4) & 0xFF;
```

Neutral center: `0x800` (2048) for both axes. Range: 0x000–0xFFF.

## SPI Flash Read Emulation

The Switch reads several SPI flash regions during setup. An emulator must return plausible data.

### Key Addresses

| Address | Size | Content |
|---------|------|---------|
| `0x6000` | 16 | Serial number (ASCII; return 0xFF if unused) |
| `0x6012` | 1 | Device type: `0x03` = Pro Controller |
| `0x6020` | 24 | Factory IMU calibration (4 groups × 3 Int16LE) |
| `0x603D` | 9 | Factory left stick calibration |
| `0x6046` | 9 | Factory right stick calibration |
| `0x6050` | 3 | Body color (RGB) |
| `0x6053` | 3 | Button color (RGB) |
| `0x6056` | 3 | Left grip color (RGB) |
| `0x6059` | 3 | Right grip color (RGB) |
| `0x8010` | 11 | User left stick cal (magic `0xB2 0xA1` + 9 data; `0xFF` = no user cal) |
| `0x801B` | 11 | User right stick cal (magic `0xB2 0xA1` + 9 data; `0xFF` = no user cal) |
| `0x8026` | 26 | User IMU cal (magic `0xB2 0xA1` + 24 data; `0xFF` = no user cal) |
| `0x5000` | 1 | Shipment low-power state flag |

### Stick Calibration Format (9 bytes → 6 values)

```c
data[0] = (cal[1] << 8) & 0xF00 | cal[0];
data[1] = (cal[2] << 4) | (cal[1] >> 4);
data[2] = (cal[4] << 8) & 0xF00 | cal[3];
data[3] = (cal[5] << 4) | (cal[4] >> 4);
data[4] = (cal[7] << 8) & 0xF00 | cal[6];
data[5] = (cal[8] << 4) | (cal[7] >> 4);
```

**Left stick order**: X_max_delta, Y_max_delta, X_center, Y_center, X_min_delta, Y_min_delta

**Right stick order**: X_center, Y_center, X_min_delta, Y_min_delta, X_max_delta, Y_max_delta

If user calibration magic (`0xB2 0xA1`) is absent, the Switch uses factory calibration.

### IMU Calibration (24 bytes at 0x6020)

| Offset | Content |
|--------|---------|
| 0–5 | Accel XYZ origin offsets (3 × Int16LE) |
| 6–11 | Accel XYZ sensitivity coefficients |
| 12–17 | Gyro XYZ origin offsets |
| 18–23 | Gyro XYZ sensitivity coefficients |

## Rumble Data Format

Report `0x10` (rumble only, no subcommand) or bytes 2–9 of report `0x01`.

```
Bytes 0–3: Left motor
Bytes 4–7: Right motor
```

Each motor (4 bytes):

| Byte | Range | Meaning |
|------|-------|---------|
| 0 | 0x04–0xFC | High-band frequency (81.75–313.14 Hz) |
| 1 | 0x00–0xFC | High-band amplitude + freq enable (LSB) |
| 2 | 0x01–0x7F | Low-band frequency (40.87–626.28 Hz) |
| 3 | 0x40–0x72 | Low-band amplitude (0x40 = floor/off) |

**Neutral (no rumble)**: `00 01 40 40 00 01 40 40`

## Connection Info Byte

The connection info nibble (low nibble of battery/connection byte):

| Value | Meaning |
|-------|---------|
| `0x0E` | USB powered, Pro Controller |
| `0x01` | BT connected |

For USB Pro Controller emulation, use `0x8E` (battery full + USB Pro Controller).

## Implementation Checklist

1. Handle `0x80` USB commands → reply with `0x81` (phase 1)
2. After `0x80 0x04`, handle `0x01` subcommands → reply with `0x21` (phase 2)
3. Gate `0x30` input reports behind setup completion (player lights = last setup cmd)
4. Emulate SPI flash reads for calibration, colors, and device type
5. Return `0xFF` for user calibration addresses (forces factory cal fallback)
6. Reply to unknown subcommands with ACK `0x80` + subcmd_id + `0x03`
7. Timer byte in 0x30 reports must increment (not be static)
