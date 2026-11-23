---
name: zt703s-uart
description: Log multimeter readings from the ZOYI ZT-703S over its TTL UART output (the signal-generator jack, 115200 8N1) to the terminal or a CSV, with a USB-TTL adapter. Use when recording a measurement over time from the ZT-703S, logging a bench test, or when its serial output is garbled or silent.
argument-hint: [--port /dev/cu.usbserial-XXXX] [--csv out.csv]
allowed-tools: Read, Bash
---

# ZOYI ZT-703S UART Logging

In multimeter mode the ZT-703S can send its reading out of the signal-generator
jack as TTL serial text, three times a second. `scripts/zt703s_read.py`
reads it, prints one line per reading, and can append them to a CSV.

## Wiring

| Meter | USB-TTL adapter |
|---|---|
| Signal-generator jack, **big** lug (TX) | RX |
| Signal-generator jack, **small** lug (GND) | GND |

These are the two probe-compensation loops next to the USB-C port. Leave the
adapter's TX unconnected: the output is one-way. One forum user measured the
signal at 3.0 V, so a 3.3 V adapter is safe; set a selectable adapter to 3.3 V.

That GND is the oscilloscope probe ground. The manual warns not to measure
with the scope probe while the serial port is connected, since the adapter
then ties probe ground to the computer's ground.

## Turn the output on

Multimeter mode, MENU, then **F4** toggles serial output (manual, "Multimeter
Extended Functions"). It sends continuously while on, about 3 records a second.

## Read

```sh
uv run --script .claude/skills/zt703s-uart/scripts/zt703s_read.py
uv run --script .claude/skills/zt703s-uart/scripts/zt703s_read.py --csv tmp/meter.csv
```

With one USB serial adapter attached the port is found by itself; with more,
the script lists them and asks for `--port`. Output, one reading per line:

```
2026-11-23T14:02:11 Voltage      -23.211 mV
2026-11-23T14:02:11 Resistance   OL KOM
```

The CSV gets `time,label,value,unit,raw`; `value` is empty for an overload or
a blank display.

## The record format

ASCII, `Label:value unit`, with **no CR or LF between records** (EEVblog
reply #139). The script ends a record at a 50 ms quiet gap, and also splits a
chunk holding several records before each `Label:`. Records seen in the forum
sample:

```
Voltage:-23.211 mV   Resistance:002.15 OM   Resistance:0.L KOM
beep:000.04 OM       Diode:00.001 V         Electricity:0.0000 A
```

- `OM` is ohms and `KOM` kilo-ohms; `0.L` is overload; `beep:. OM` and
  `Diode:. V` are the open-circuit display.
- The record does not say AC or DC. Note the switch position yourself.
- This sample is one user's capture on 2023-era firmware. Other modes and
  later firmware may use other labels; the script prints a record it cannot
  parse as `? '<record>'` on stderr instead of dropping it. Add new labels
  and units here when they are seen.

## Garbled or silent

- **Nothing arrives.** Check F4 is on, the meter is in multimeter mode, and RX
  is on the big lug. A charge-only adapter cable carries no data.
- **Garbage characters.** Run with `--raw` to see the bytes as hex. A wrong
  baud gives framing noise; try the adapter on another known 115200 source.
  One forum user only ever got garbled output from their adapter.
- **Binary frames starting `FF`, at 9600 baud.** That is not this output. The
  [esp32_zoyi703s_esp32](https://github.com/luubinhng/esp32_zoyi703s_esp32)
  project taps pads inside the meter, which carry 18-byte LCD-segment frames
  at 9600 baud; its `src/main.cpp` decodes them. This skill does not.

## Sources

- ZOYI ZT-703S owner's manual, "Multimeter Extended Functions" (F4, 115200,
  3 per second, ground warning):
  <https://www.manualslib.com/manual/3351110/Zoyi-Zt-703s.html?page=32>
- EEVblog ZT-70xS thread, page 6, replies #139 (sample, no terminator, 3.0 V),
  #141 (garbled adapter), #142–#143 (115200 8N1):
  <https://www.eevblog.com/forum/testgear/new-zoyi-multimeteroscilloscopes-zt-70xs-up-to-50mhz250msps-(nov-2022)/125/>
- Same thread, page 7, replies #150–#153 (lugs):
  <https://www.eevblog.com/forum/testgear/new-zoyi-multimeteroscilloscopes-zt-70xs-up-to-50mhz250msps-(nov-2022)/150/>

Not yet checked against this meter: the script has been run only against the
forum's sample text, not a live capture.
