# robocar-unified: Camera frames, sensor and the J15 log mirror

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

## Seeing what the camera actually sends

Every claim about image quality here was inference until `frame_dump.c` existed — the sensor registers, the JPEG size and the frames themselves had never been read off a device. Two instruments, and the pairing is the diagnosis:

- **`frame_stats_log()` runs on every planner frame.** It decodes the JPEG at 1/8 scale and logs `luma mean/p5/p50/p95` next to the sensor's **live** `gain`/`exp`/`ceil` registers. Read them together: high gain + maxed exposure + low luma means the sensor is *starved* and the gain ceiling needs raising; low gain + low exposure + low luma means auto-exposure is not converging; low mean with a **high p95** is backlit, not unlit. The cached `sensor_t` status struct cannot answer this — it holds what was last *written*, not what the AEC/AGC loops have since chosen, which is why `camera_read_exposure()` reads the registers over SCCB instead. The same decode also yields the scene fingerprint that gates speech (see the Voice section), which is why it is *returned* rather than acted on here: this file is instrumentation, and what a frame means for the robot's behaviour is the planner's call.
- **`frame_dump_maybe()` emits the JPEG itself**, base64-framed with a length and CRC32, for the first `FRAME_DUMP_BOOT_FRAMES` frames and on `snap [n]`. These are the exact bytes handed to `gemini_backend_plan()`, dumped while the planner still holds the buffer — no copy, no lifetime question, no chance of showing a different frame from the one the model saw.

```bash
just robocar-unified::monitor | tee /tmp/robocar.log
uv run tools/decode-frame-dump.py /tmp/robocar.log /tmp/frames
```

The monitor is **read-only** and resets the board on attach (`tools/esp32s3-monitor.sh` never writes to the port), so you cannot type `snap` through it — that is why the first frames auto-dump at boot. Use `screen`/`picocom` if you want the command.

The length+CRC are not decoration: the USB-Serial-JTAG console **silently drops** TX bytes once the host stops reading for 50 ms (`TX_FLUSH_TIMEOUT_US`) and reports success anyway. Without the checksum a host hiccup yields a corrupt JPEG that looks exactly like a broken camera.

The dumps come over USB only. They are `printf()` lines, not `ESP_LOG`, so the UART0 mirror below never sees them — which is deliberate: a 20 kB JPEG is ~27 kB of base64, over two seconds of 115200-baud wire time, and would push every log line behind it out of the mirror's queue.

### The log on J15 (UART0 TX)

`log_mirror.c` (issue #798) installs an `esp_log_set_vprintf()` hook as the first thing in `app_main`. Each `ESP_LOG` line still goes to the previous handler — the USB-Serial-JTAG console, unchanged — and a copy goes to UART0 TX on D6 (GPIO43) at `LOG_MIRROR_UART0_BAUD` (115200 8N1, the ROM boot log's own rate), which the carrier routes to test header J15. That is the log on a wire that does not drop bytes when a USB host stops reading, and that is there off the tether. The console stays on USB: `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` is untouched, `command_task` still reads stdin from USB, and Improv packets, command replies and the frame/mic dumps — all written to stdout, not logged — never reach UART0. D7 (RX) is not claimed (`UART_PIN_NO_CHANGE`, RX interrupt off).

The hook never waits on the wire. It renders the line into a `LOG_MIRROR_LINE_MAX`-byte buffer on the logging task's stack (`log_mirror_core.c`: LF→CRLF, since no VFS sits between the driver and the pin, and a cut line ends in `...\r\n`), then offers it to a PSRAM NOSPLIT ring buffer with a zero timeout. A full ring drops the line and counts it; a priority-1 drain task on core 1 is the only caller of `uart_write_bytes()`, and prints `[log_mirror: N line(s) dropped]` on J15 when it catches up. It does not ask the UART driver for free space first because `uart_get_tx_buffer_free_size()` in v5.4 subtracts only the item the ISR is sending, not what is queued behind it. `LOG_MIRROR_UART0_ENABLED 0` in `config.h` compiles it out. `test_log_mirror_core.c` pins the rendering (bounded, marked when cut, CRLF not doubled); how many lines a boot burst drops is bench work on issue #798.

**The stack cost was measured the hard way (2026-11-04, #939).** The hook formats every line once into its buffer and the previous handler formats it again, roughly doubling what an `ESP_LOG` call costs on the calling task's stack. On the FreeRTOS timer task (2048 bytes) that overflowed: the OTA stability callback logs twice 60 s after boot, and the board rebooted every 60–75 s. `CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH` is now 4096. Any small-stacked task that logs needs the same check.

**A crash report never reaches J15.** The stack-overflow report and panic dumps are written by the panic handler, not through `esp_log_set_vprintf()`, so they appear on the USB console only (2026-11-04: `***ERROR*** A stack overflow in task Tmr Svc` was on USB and absent from every J15 capture). A board rebooting with no panic in the J15 capture is not evidence of a power fault: read the USB console before theorising about supply.

**J15 does carry the reset reason, twice.** The ROM boot log that follows any reset goes to UART0 at the same 115200 baud, and its third line names the cause (`rst:0xf (BROWNOUT_RST)`, `rst:0x1 (POWERON)`, `rst:0x15 (USB_UART_CHIP_RESET)` after a flash or `just reset`). Then the first mirrored line after the banner is `Boot: reset <reason> (<n>) | robocar-unified v<version> build <sha>` (#986). `BROWNOUT` is power, as is `POWERON` with no deliberate power-up (the supply dropped out entirely); `PANIC`, `TASK_WDT`, `INT_WDT` or `SW` is firmware, and its dump is on USB. On 2026-11-07 this split 20 battery reboots onto the supply: every one was `BROWNOUT_RST`, each within milliseconds of a goal that starts the wheels, which a multimeter cannot see (#991).

**Capture it whole, and grep it with `-a`.** Open the adapter once with DTR and RTS deasserted (so the adapter cannot reset the board), keep reading across resets, and prefix host timestamps, so a reset lands between the line it interrupted and the ROM log after it. The capture starts with ROM bytes that are not text, so plain `grep` treats the file as binary and prints no matching lines at all: a watch filter on it stayed silent through two real resets until it ran `grep -a`.

### The sensor is an OV3660, not an OV2640

`camera_pins.h` claimed OV2640 for years and it is **wrong** — the board reports `PID=0x3660`. The first frame this instrument ever dumped is what revealed it, and it inverted the diagnosis:

`set_gainceiling()` means completely different things on the two sensors. On an OV2640 the argument is an enum index (`0` = 2x, the lowest of seven). On an OV3660 the driver writes it as a **raw 10-bit ceiling** into registers `0x3A18`/`0x3A19`, which esp32-camera's OV3660 init table sets to `0x00F8` = 248 = **15.5x** (the chip's own reset value differs; see [the OV3660 page](../../../../../docs/reference/datasheets/camera--ov3660.md)). `camera_init()` was passing `(gainceiling_t)0`, so on this hardware it was not selecting "2x" — it was clamping the AGC ceiling to **zero gain**. That is why the frames were dark. The call is gone; the driver's own default stands.

The register maps are otherwise incompatible too (OV2640 is 8-bit-addressed and bank-switched, OV3660 is 16-bit-addressed), so `camera_read_exposure()` branches on the PID and returns `false` for anything it does not know rather than reading one sensor's map through another's driver. **Read the PID `camera_init()` logs before reasoning about any sensor register.**

Exposure stays a **runtime knob** (`cam gainceiling N`), not a compile-time constant, because whether a frame is "too dark" is a judgement only a human looking at the room can make and a reflash per trial is far too slow a loop. Its range and units are sensor-dependent — `cam` with no argument prints both.


## Don't

- Don't set `fb_count = 1` on the camera. Capture halts while the app holds the only buffer, so the planner's frame is one full period (15 s) old — the robot plans motion from a stale view, and `grab_mode` is inert below 2 buffers
- Don't re-add `set_aec_value()` / `set_agc_gain()` alongside `set_exposure_ctrl(1)` / `set_gain_ctrl(1)` — the sensor's own loops rewrite those registers every frame, so the calls do nothing but read like deliberate tuning. `set_agc_gain(s, 0)` is worse than nothing: it writes the *minimum* gain
- Don't call `set_gainceiling()` with an OV2640-shaped `0..6` — this board is an OV3660, where the argument is a raw 10-bit value and `0` means *no gain at all*. That one line is what made the frames dark. Check the logged PID first
- Don't read sensor registers without branching on the PID. One sensor's map read through another's driver returns confident, plausible, meaningless numbers — `gain=0 exp=0` looked like a real "auto-exposure isn't converging" diagnosis until the PID was printed
- Don't flip `set_aec2()` on the strength of web folklore — the vendored driver's setter writes the inverse of its argument while its getter reads the raw bit, so even the direction of the change is unresolved in the source. A/B it against measured luma or leave it
- Don't change camera core pinning without re-verifying motor PWM jitter
- Don't call `ESP_LOG` from inside `log_mirror.c`'s vprintf hook or its drain task, and don't make the hook wait on the UART. The hook runs on every task that logs; a log line from inside it re-enters it, and a blocking write throttles the 30 Hz executor to 115200 baud whenever something logs a burst. Drop and count instead, as it does now
- Don't move the console to UART0 to "simplify" the mirror. `improv_console.c` refuses to compile without `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`, ESP Web Tools provisions over the USB-C port, and the mirror exists precisely so the console can stay where it is
