# LEGO Boost ⟵ Xbox Controller

Drive a **LEGO Boost Creative Toolbox (set 17101)** Move Hub model — Vernie /
MTR4 — with an **Xbox Series X|S controller**.

## Why a middleman is required

The Boost Move Hub speaks the **LEGO Wireless Protocol (LWP) v3** over Bluetooth
Low Energy. Both the hub *and* the Xbox controller are BLE **peripherals** — the
hub advertises and waits to be connected to, and the controller advertises as an
HID peripheral. Two peripherals cannot connect to each other; a BLE **central**
must sit in the middle.

> **Can't I just pair the Xbox controller straight to the hub?**
> No — not with the 17101. Pybricks firmware *can* turn a hub into a central that
> hosts an Xbox controller, but only on the **Technic** and **City** hubs. The
> Move Hub's ~106 KiB of free flash is too small for that feature (and too small
> even for stdin-over-BLE). So the 17101 structurally needs a PC or microcontroller
> in the middle.

## Two architectures (we keep both)

```
A) Host PC bridge   Xbox ──BT-HID──▶ browser ──Web Bluetooth/LWP──▶ Move Hub (stock fw)
C) Two ESP32        Xbox ──BT──▶ ESP32[Bluepad32] ──UART──▶ ESP32[legoino] ──BLE──▶ Move Hub
```

- **Option A (`web/`) — implemented here.** A zero-build browser app: the Gamepad
  API reads the controller, [`node-poweredup`](https://github.com/nathankellenicki/node-poweredup)
  drives the hub over Web Bluetooth. Fastest path to a moving robot; needs a
  laptop in the loop. Works on macOS with no drivers.
- **Option C — planned.** Standalone, no laptop. Reuses the Bluepad32 pattern from
  the sibling [`xbox-switch-bridge`](../xbox-switch-bridge/) package plus a second
  ESP32 running [legoino](https://github.com/corneliusmunz/legoino) as the BLE
  central to the hub, joined over UART (the robocar bridge pattern). The two-chip
  split deliberately sidesteps the unproven single-chip dual-BLE-central path
  (Option B).

## Run Option A

Requirements: **Chrome or Edge** (Web Bluetooth), an Xbox Series controller, the
Move Hub powered on.

```sh
just serve            # static server at http://localhost:8000
```

Then, in Chrome/Edge:

1. Pair the Xbox controller to the computer over Bluetooth (it appears as a
   standard HID gamepad — no driver needed on macOS/Linux).
2. Open <http://localhost:8000/web/>.
3. Click **Connect Hub** and pick the Move Hub in the Bluetooth picker (press the
   hub's green button if it isn't advertising).

### Controls

| Input | Action |
|---|---|
| Left stick (Y) | Drive forward / backward |
| Left stick (X) | Turn (arcade mix) |
| Right stick (X) | Turn the head left / right |

### Vernie (17101) port map

Confirmed from the live device inventory the app logs on connect:

| Port | Device | Role |
|---|---|---|
| A | built-in motor (type 39) | Left track |
| B | built-in motor (type 39) | Right track |
| C | color/distance sensor | unused (future: line-follow / bump) |
| D | external motor (type 38) | Head |

The app still logs the full inventory on every connect and will auto-pick
drivable ports if your build differs.

## Tuning

All knobs are at the top of [`web/boost-xbox.js`](web/boost-xbox.js):

- **Drive ports** `A`/`B` are Vernie's built-in track motors. The two tracks are
  mounted mirrored — if Vernie spins in place when you push forward, flip
  `RIGHT_INVERT`.
- **`ARM_PORT`** (`D` by default) is the external head motor; set it to `null` if
  your build has none.
- **`DEADZONE`**, **`MAX_POWER`** tune stick feel and top speed.

## Notes & gotchas

- **node-poweredup's known motor caveat** affects only *external* Control+/SPIKE
  motors (the hub ignores their commands pending a LEGO firmware fix). The
  **built-in** Move Hub motors used by Vernie's tracks are a different device
  class and work fine.
- The bridge uses the **vendored self-contained browser bundle** at
  `web/vendor/poweredup.js` (node-poweredup 10.1.0's `dist/browser/poweredup.js`),
  loaded via a classic `<script>` tag — it assigns `window.PoweredUP`. We do *not*
  load it from a CDN like esm.sh: esm.sh resolves the package's **Node** entry and
  drags in the `usb`/`serialport` native deps (hundreds of 404/500s, no Web
  Bluetooth). To update the vendored bundle:
  `curl -sL https://unpkg.com/node-poweredup@10.1.0/dist/browser/poweredup.js -o web/vendor/poweredup.js`.
- Web Bluetooth needs a secure context; `http://localhost` qualifies, so no HTTPS
  is needed for local use.

## References

- [LEGO Wireless Protocol (official)](https://lego.github.io/lego-ble-wireless-protocol-docs/)
- [node-poweredup](https://github.com/nathankellenicki/node-poweredup) — host-PC library (active)
- [Pybricks Move Hub docs](https://docs.pybricks.com/en/latest/hubs/movehub.html)
- [Bluepad32](https://github.com/ricardoquesada/bluepad32) — Xbox controller host for Option C
- [legoino](https://github.com/corneliusmunz/legoino) — ESP32 LWP central for Option C
