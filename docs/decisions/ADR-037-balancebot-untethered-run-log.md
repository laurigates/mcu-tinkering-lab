# ADR-037: Balancebot Untethered Run Log in RAM, Saved to Flash After the Run

**Status**: Accepted
**Date**: 2026-11-24
**Confidence**: 6/10 (host-tested record, ring, session and header; the dump
path tested against a fake board on a pty; not yet run on the robot, and the
flash timings are typical figures, not measured on the XIAO's chip)
**Extends**: [ADR-018](ADR-018-pico-sdk-balancebot.md)

---

## Context

Every tuning measurement on the balancebot
([PRD-012](../requirements/PRD-012-balancebot.md)) is a USB capture: `stream on`
prints CSV telemetry over USB CDC and `just balancebot::capture` records it. The
robot has to stay on the cable for that, and the cable pulls on the robot. The
measurements that matter most now are the ones a cable spoils. #1262's sustained
push needs the robot to roll freely until it falls, so that the capture shows
whether `rate_sps` sits at `maxrate` first. That decides between raising `q_x`
or `maxrate` and a 4S pack.

Four constraints shaped the design:

- **Flash writes stall both cores.** An erase or program stops XIP, so core 1's
  500 Hz loop cannot run during one. `CMD_SAVE` is refused in RUN for this
  reason (ADR-018, `balance_control.c`).
- **The watchdog.** Once the robot has armed, a 100 ms watchdog
  (`WATCHDOG_TIMEOUT_MS`) runs. Core 1 feeds it, and core 1 is locked out for
  the length of each flash operation.
- **Reading a log out means power-cycling the robot.** USB only enumerates if
  the cable goes in before the pack (#1230), so the usual way to read a log is
  to unplug the pack, plug in USB, and reconnect the pack. A log held only in
  RAM is lost at that power cycle.
- **There is room.** The XIAO RP2350 has 512 KB of main SRAM and 2 MB of flash.
  Before this change the firmware used about 21 KB of RAM (`.data`, `.bss`,
  heap minimum and stacks, from `llvm-size -A build/balancebot.elf`) and about
  115 KB of flash (124 KB with this change, `__flash_binary_end`).

## Decision

Record each run into a RAM ring on core 0. Once the run has ended and the
motors are off, copy it to a fixed flash region. `log dump` prints it back as
the same CSV `stream` produces.

### Recording

- **Who samples**: core 0, from the seqlocked snapshot core 1 already
  publishes for telemetry. Core 1 does not change. Default 100 Hz
  (`LOG_DEFAULT_HZ`), up to 200 Hz (`LOG_MAX_HZ`, telemetry's cap). Sampling is
  held to the nominal rate and resyncs after a stall instead of bursting.
- **Row**: the nine telemetry columns in binary, 32 bytes, so 128 rows fill a
  4 KB sector (`log_row_t`, `src/log_record.h`).
- **Ring**: 8192 rows (`LOG_CAPACITY_ROWS`), 256 KB of RAM: 82 s at 100 Hz,
  41 s at 200 Hz. A longer run keeps its newest rows, so the fall survives.
  The header counts the dropped rows.
- **Session** (`log_session_sample()`, pure C):
  - It starts at the first sample that reads RUN and clears the ring then.
  - It ends `LOG_TAIL_S` (1 s) after the first sample that does not read RUN,
    so the fall and the robot coming to rest are in the log.
  - A run that ends with `disarm` saves the same way.
  - If the robot re-arms during the tail, the run before it is dropped
    (`LOG_ACTION_LOST`, counted in `log`), because flash cannot be written in
    RUN.
- **Default on.** An untethered run should not depend on having remembered a
  command; `log off` turns it off. The setting is not persisted.

### Saving

`log_store_save()` (`src/log_store.c`), called from core 0 when a session
ends:

1. **Hold off the auto-arm gesture** for 1 s plus 0.1 s per sector, and wait
   for core 1 to apply it. Core 0 is busy for the whole save, so `arm` cannot
   arrive. With the gesture held off, nothing else can enter RUN either.
2. **Pick a slot and erase its header sector.** The save writes into the
   first slot that holds no valid run, or else the one with the oldest save
   sequence number (`log_slots_next()`, `src/log_record.c`). From here until
   step 4, a power cut leaves that slot empty rather than a mix of two runs;
   the other five keep theirs.
3. **Write the rows** one sector at a time: an erase, then a program of only
   the pages used. Each is its own `flash_safe_execute()` call, with
   `watchdog_update()` before it. A sector is about 45 ms of erase and a few ms
   of programming (typical figures). Before each operation the save checks
   that the robot is not armed, and stops if it is.
4. **Write the header last**: magic, version, row count, rate, rows
   overwritten, a save sequence number, the state that ended the run, a CRC32
   over the rows and one over the header.
5. **Read back** the header and the rows' CRC through XIP.

**Region**: six slots of 65 sectors (260 KB) each, 1560 KB directly below the
param sector (`LOG_FLASH_OFFSET`, from 0x78000). It is not in the last block,
which the RP2350-E10 UF2 workaround erases. It sits above the firmware image
(which ends near 0x1F700), and `log_store_fits()` refuses a save if the image
ever grows into it. The newest six runs are kept (#1277, 2026-11-26). Until
then the region was one slot; that slot is now the top one, so a run saved by
the older firmware reads as one of the six.

### Reading

- `log` shows the state: on or off, rate, recording or not, the last save's
  result, and the stored log with its rows CRC checked. `stat` shows the same
  without the CRC check.
- `log list` prints one line per slot that holds a run, newest first, with its
  run number (the save sequence number), rows, rate and how it ended.
- `log dump` (disarmed only) prints the newest run as a `#` summary line, the
  CSV header, the rows, and `log end`. `log dump <n>` prints run `n` from
  `log list`.
- `just balancebot::log-dump [n]` records the dump (run `n` if given) to
  `tmp/balancebot/log-<time>.csv`. It uses `tools/serial-capture.py` with
  `--send "log dump" --until "log end"`, and fails without writing a file if
  the end marker does not arrive. `analyze` reads the result unchanged.
- `stream` and `log dump` format rows through the same `log_row_format_csv()`,
  so the two cannot drift apart.

## Consequences

**Positive**

- A push-and-fall, or any run away from the bench, is recorded with no extra
  hardware. The last 82 s at 100 Hz survive a power cycle.
- The output is the CSV every existing tool reads (`analyze`, `fit-leff`).
- Core 1 and its timing do not change.
- The row format, the ring, the session's start, tail and loss rules, and the
  header framing are host-tested (`tests/host/test_log_record.c`).

**Negative**

- **Saving stalls core 1** about 50 ms per flash operation while the robot is
  in IDLE or FAULT. The IMU filter misses those samples. The robot is lying
  down or held, and the filter's ~0.5 s time constant recovers within the
  hold-off.
- **The CLI and LED pause during a save**: about 0.5 s for a 10 s run at
  100 Hz, up to about 4 s for a full ring.
- **The auto-arm gesture is held off** for the save's estimate, up to about
  7.5 s for a full ring.
- **A sector erase can outlast the watchdog.** The worst-case erase time of a
  NOR flash sector is several times the typical 45 ms, beyond the 100 ms
  watchdog. `CMD_SAVE` already carries the same exposure. A reboot mid-save
  loses that log and nothing else: the motors are off and the external
  nENABLE pull-up keeps them off.
- **A run that ends in a brownout or a BMS cutoff is lost.** No state change
  is seen, so nothing is saved. #747, a low-pack fault on the carrier's
  divider, would turn most of those into a FAULT that saves.
- **Rows are sampled by core 0, not taken on control ticks.** A blocking CLI
  command leaves a gap, visible in `t_ms`. This matches what a USB capture has
  always had.
- **Flash wear.** Each saved run erases its slot's header sector and the
  sectors its rows use. The slots take turns, so typical NOR endurance (about
  100k cycles per sector) allows hundreds of thousands of runs.

## Alternatives Considered

- **RAM only**: rejected. Because of #1230, reading out usually means a power
  cycle, which loses the log.
- **Write flash continuously during RUN**: rejected. Every erase or program
  stalls core 1 mid-balance. Avoiding that means running core 1's loop and
  everything it calls from RAM and locking XIP out per write, which reshapes
  ADR-018's core split for a debugging feature.
- **Record on core 1 at every tick**: rejected for now. It would give
  tick-exact rows, but it adds work and a hand-off on the hard real-time core.
  Core 0's sampling at up to 200 Hz already resolves a ~1 Hz swing and a
  half-second fall.
- **A UART microSD logger (OpenLog-style)**: rejected for now. It removes the
  length limit but adds a module and wiring. J6's D1/D2 are planned for the
  co-processor link (#1218).
- **Stream over Wi-Fi through the planned ESP32-S3 co-processor**: the
  long-term route, blocked on #1217's choice. The CSV format chosen here is
  what it would send.

## Related

- [ADR-018](ADR-018-pico-sdk-balancebot.md): the core split and the
  no-flash-in-RUN rule this works within
- [ADR-026](ADR-026-balancebot-lqr-live-switch.md): the CSV columns
- [PRD-012](../requirements/PRD-012-balancebot.md): the balancebot requirements
- #1262 (the push test this was built for), #1230 (USB/pack order), #747
  (pack-voltage fault)
- `packages/robotics/balancebot/src/log_record.{c,h}`, `log.{c,h}`,
  `log_store.{c,h}`, `tests/host/test_log_record.c`
