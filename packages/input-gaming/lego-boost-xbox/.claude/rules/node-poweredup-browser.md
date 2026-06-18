---
description: Gotchas for driving LEGO Powered UP / Boost hubs from a browser via node-poweredup + Web Bluetooth
globs: ["packages/input-gaming/lego-boost-xbox/**"]
---

# node-poweredup in the Browser (Web Bluetooth)

Running [`node-poweredup`](https://github.com/nathankellenicki/node-poweredup)
in a browser to drive a LEGO hub (Boost Move Hub, Technic, City…) hits four
non-obvious traps. All four bit during the initial bring-up of this package;
each fix is load-bearing.

## 1. Do NOT load it from a JS-bundling CDN (esm.sh/jspm)

`esm.sh` resolves the package's **Node** entry and drags in the `usb` and
`serialport` native deps — the browser console fills with dozens of 404/500s
for `usb-darwin-arm64`, `serialport@^12`, etc., and there is no Web Bluetooth
transport. Symptom: a storm of `Failed to load resource: 404/500` before any
hub code runs.

**Fix:** vendor the self-contained prebuilt browser bundle and load it with a
classic `<script>` tag:

```sh
curl -sL https://unpkg.com/node-poweredup@10.1.0/dist/browser/poweredup.js \
  -o web/vendor/poweredup.js
```

```html
<script src="./vendor/poweredup.js"></script>
```

## 2. The bundle's global is `window.PoweredUP` — a namespace, not the class

`dist/browser/poweredup.js` is an IIFE that assigns
`window.PoweredUP = { PoweredUP, BaseHub, WeDo2SmartHub, … }`. The constructor
is therefore `window.PoweredUP.PoweredUP`, not `window.PoweredUP`:

```js
const { PoweredUP } = window.PoweredUP;
const hub = new PoweredUP();
```

(Some sources claim the bundle exposes no global — it does. Verify by grepping
the vendored file for `window.PoweredUP =`.)

## 3. Shim Node's `process` before the bundle loads

The motor-command path still references `process` (`process.env`,
`process.nextTick`, `process.type`), so `motor.setPower()` throws
`ReferenceError: process is not defined` in the browser. Inject a minimal shim
**before** the `<script src=poweredup.js>`:

```html
<script>
  window.process = window.process || {
    env: {}, type: "browser",
    nextTick: (fn, ...a) => setTimeout(() => fn(...a), 0),
  };
</script>
```

## 4. Discover motor ports from a live inventory, not from assumptions

Port numbers/labels vary by hub and build, and an attached **sensor** at a port
makes a naive `waitForDeviceAtPort` "succeed" with a non-motor device (then
`setPower` is `undefined`). On the Boost Move Hub running Vernie (set 17101) the
real map is: **A/B = built-in track motors, C = color/distance sensor,
D = external head motor**.

Robust pattern, used in `web/boost-xbox.js`:

- After `connect()`, wait ~1.5 s for async device attach, then enumerate
  `getDeviceAtPort("A".."D")` and log each `typeName`/`type`.
- Treat a device as drivable only if `typeof d.setPower === "function"`
  (BasicMotor subclasses); skip sensors.
- `waitForDeviceAtPort` hangs forever on a **reconnect** when the device is
  already attached (its event already fired) — check `getDeviceAtPort` first,
  then fall back to a timeout-guarded wait.

## Gamepad side (HTML5 Gamepad API)

- Don't latch a single `gamepad.index` from the `gamepadconnected` event — Xbox
  pads fire duplicate connects (USB+BT) and sleep/reconnect. Pick the first live
  `standard`-mapping pad every frame in the `requestAnimationFrame` poll loop.
- Standard mapping: left stick `axes[0]/[1]`, right stick `axes[2]/[3]`,
  triggers `buttons[6]/[7].value`, A/B `buttons[0]/[1].pressed`.

## Bandwidth

BLE write bandwidth is precious — only send `setPower` when the value changed
meaningfully (e.g. ≥3 units), and zero the motors when the controller drops so
the robot can't run away.
