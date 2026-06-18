// Option A bridge: Xbox Series controller (HTML5 Gamepad API) -> LEGO Boost
// Move Hub (Web Bluetooth via node-poweredup). Browser-only, zero native deps.
//
// node-poweredup is loaded as a classic <script> in index.html (the vendored
// self-contained browser bundle web/vendor/poweredup.js), which assigns the
// global `window.PoweredUP = { PoweredUP, BaseHub, ... }`. We grab the class
// from that namespace. (esm.sh resolves the Node entry and drags in usb/
// serialport native deps, so it cannot be used here — see README.)
const { PoweredUP } = window.PoweredUP;

// --- Tuning knobs ---------------------------------------------------------
// Vernie/MTR4 built-in drive motors live on ports C and D (confirmed against
// node-poweredup's movehub portMap). The two track motors are mounted
// mirrored, so one must be inverted for the robot to drive straight. If Vernie
// spins in place when you push the stick forward, flip RIGHT_INVERT.
// Vernie (set 17101) port map, confirmed from the live device inventory:
//   A = left track  (built-in MOVE_HUB_MEDIUM_LINEAR_MOTOR, type 39)
//   B = right track (built-in MOVE_HUB_MEDIUM_LINEAR_MOTOR, type 39)
//   C = color/distance sensor (not a motor)
//   D = head (external MEDIUM_LINEAR_MOTOR, type 38)
const DRIVE = {
  LEFT_PORT: "A",
  RIGHT_PORT: "B",
  RIGHT_INVERT: false, // tracks are mirror-mounted; flip if forward => spin
  LEFT_INVERT: false,
};
// Vernie's head motor (external, port D). Controlled by the right stick X for
// look-around. Set to null to disable, or another port to remap.
const ARM_PORT = "D";
const DEADZONE = 0.12; // ignore stick noise near center
const MAX_POWER = 100; // node-poweredup setPower range is -100..100

// --- State ----------------------------------------------------------------
let leftMotor = null;
let rightMotor = null;
let armMotor = null;
// Avoid spamming the hub with identical commands (BLE bandwidth is precious).
let lastSent = { left: null, right: null, arm: null };

const statusEl = () => document.getElementById("status");
const log = (msg) => {
  console.log(msg);
  const el = document.getElementById("log");
  if (el) el.textContent = `${new Date().toLocaleTimeString()}  ${msg}\n` + el.textContent;
};
const setStatus = (text, cls) => {
  const el = statusEl();
  el.textContent = text;
  el.className = cls || "";
};

// --- Hub connection -------------------------------------------------------
const poweredUP = new PoweredUP();

// A device is "drivable" if it exposes setPower (BasicMotor and subclasses,
// incl. the Move Hub's built-in MoveHubMediumLinearMotor). Sensors do not.
const isDrivable = (d) => d && typeof d.setPower === "function";

// Log every attached device so we can see exactly what's on each port and
// which ones are motors vs sensors. Devices attach asynchronously after
// connect, so give them a moment first.
async function inventoryPorts(hub) {
  await new Promise((r) => setTimeout(r, 1500));
  const found = {};
  for (const port of ["A", "B", "C", "D"]) {
    const d = hub.getDeviceAtPort(port);
    if (d) {
      found[port] = d;
      log(`port ${port}: ${d.typeName ?? "?"} (type ${d.type})${isDrivable(d) ? " [drivable]" : ""}`);
    } else {
      log(`port ${port}: empty`);
    }
  }
  return found;
}

poweredUP.on("discover", async (hub) => {
  log(`Discovered hub: ${hub.name} (${hub.type})`);
  await hub.connect();
  log("Hub connected.");
  setStatus("Hub connected — scanning ports…", "ok");

  const found = await inventoryPorts(hub);

  // Prefer the configured drive ports, but only if they're actually motors.
  // Otherwise auto-pick the drivable ports so a wrong port guess can't brick it.
  const drivablePorts = ["A", "B", "C", "D"].filter((p) => isDrivable(found[p]));
  let lp = isDrivable(found[DRIVE.LEFT_PORT]) ? DRIVE.LEFT_PORT : drivablePorts[0];
  let rp = isDrivable(found[DRIVE.RIGHT_PORT]) ? DRIVE.RIGHT_PORT : drivablePorts.find((p) => p !== lp);

  if (!lp || !rp) {
    log(`Found ${drivablePorts.length} drivable motor(s): [${drivablePorts.join(", ") || "none"}]. Need 2 for tank drive.`);
    setStatus("Couldn't find two drive motors — see the port list in the log.", "err");
    return;
  }
  leftMotor = found[lp];
  rightMotor = found[rp];
  log(`Drive: LEFT=port ${lp}, RIGHT=port ${rp}.`);

  // Arm = a drivable port that isn't a drive motor (skip if none).
  const armPort = drivablePorts.find((p) => p !== lp && p !== rp);
  if (armPort) {
    armMotor = found[armPort];
    log(`Arm motor on port ${armPort}.`);
  } else {
    armMotor = null;
    log("No spare motor for the arm — arm control disabled.");
  }

  const pad = firstGamepad();
  setStatus(
    pad
      ? "Driving — left stick to move, triggers for the arm."
      : "Motors ready, but NO controller detected. Press a button on the Xbox pad.",
    pad ? "ok" : "warn",
  );

  hub.on("disconnect", () => {
    log("Hub disconnected.");
    leftMotor = rightMotor = armMotor = null;
    setStatus("Hub disconnected. Click Connect to reconnect.", "warn");
  });
});

async function connectHub() {
  if (!navigator.bluetooth) {
    setStatus("Web Bluetooth unavailable. Use Chrome/Edge over http://localhost or https.", "err");
    return;
  }
  setStatus("Select your Move Hub in the Bluetooth picker…", "");
  log("Scanning… (press the green button on the Move Hub if it isn't advertising)");
  // scan() opens navigator.bluetooth.requestDevice — must be a user gesture.
  poweredUP.scan();
}

// --- Gamepad handling -----------------------------------------------------
// Don't trust a single stored index: the Xbox pad commonly fires duplicate
// connect events (USB+BT) and may sleep/reconnect. Instead pick the first live
// gamepad every frame, so the loop self-heals across drops and reconnects.
function firstGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const p of pads) if (p && p.connected && p.mapping === "standard") return p;
  for (const p of pads) if (p && p.connected) return p; // non-standard fallback
  return null;
}

let lastPadSeen = false;
window.addEventListener("gamepadconnected", (e) => {
  log(`Gamepad connected: ${e.gamepad.id} (mapping: ${e.gamepad.mapping})`);
});
window.addEventListener("gamepaddisconnected", (e) => {
  log(`Gamepad disconnected: ${e.gamepad.id}`);
});

const clamp = (n, lo, hi) => Math.max(lo, Math.min(hi, n));
const dz = (v) => (Math.abs(v) < DEADZONE ? 0 : v);

// Send power only when it changed enough to matter (>=3 units), to spare BLE.
function drive(motor, power, key) {
  if (!motor || typeof motor.setPower !== "function") return;
  const p = Math.round(clamp(power, -MAX_POWER, MAX_POWER));
  if (lastSent[key] !== null && Math.abs(lastSent[key] - p) < 3 && !(p === 0 && lastSent[key] !== 0)) {
    return;
  }
  lastSent[key] = p;
  motor.setPower(p); // fire-and-forget; correct for continuous control
}

function pollLoop() {
  const gp = firstGamepad();
  // Surface controller appear/disappear in the log without spamming.
  if (!!gp !== lastPadSeen) {
    lastPadSeen = !!gp;
    if (!gp && leftMotor) {
      // Controller vanished mid-drive: coast the motors so it doesn't run away.
      drive(leftMotor, 0, "left");
      drive(rightMotor, 0, "right");
      log("Controller lost — motors stopped. Press a button to resume.");
    }
  }
  if (gp && leftMotor && rightMotor) {
    // Arcade drive from the left stick: up = forward, X = turn.
    const forward = -dz(gp.axes[1]); // axis Y is inverted (up is negative)
    const turn = dz(gp.axes[0]);
    let left = (forward + turn) * MAX_POWER;
    let right = (forward - turn) * MAX_POWER;
    if (DRIVE.LEFT_INVERT) left = -left;
    if (DRIVE.RIGHT_INVERT) right = -right;
    drive(leftMotor, left, "left");
    drive(rightMotor, right, "right");

    // Head: right stick X looks left/right.
    if (armMotor) {
      drive(armMotor, dz(gp.axes[2]) * MAX_POWER, "arm");
    }
  }
  requestAnimationFrame(pollLoop);
}

document.getElementById("connect").addEventListener("click", connectHub);
requestAnimationFrame(pollLoop);
log("Ready. Connect the Xbox controller (it should be detected automatically), then click Connect Hub.");
