# Custom Move Hub firmware — options for direct Xbox pairing

Research and options for writing **custom LEGO Boost Move Hub firmware** that
pairs an Xbox controller **directly** with the hub — eliminating the
host-PC / second-microcontroller middleman used by the
[`lego-boost-xbox`](../../lego-boost-xbox/) browser bridge.

> **Status:** research + feasibility. No firmware is built yet. This document
> records the hardware findings, enumerates the platforms / languages /
> frameworks available, recommends an approach, and defines a feasibility spike
> to prove direct pairing before investing in a full firmware.

## The question

Can we drop the middleman and have the Boost Move Hub itself act as the
Bluetooth **central** that connects to an Xbox controller? Pybricks proves a
hub can do this — but *not on the Move Hub*. So can custom firmware?

## Decisive findings

### 1. Pybricks does not support the Move Hub for Xbox controllers

The built-in Xbox controller feature is available on the **Technic, SPIKE
Prime/Essential, and MINDSTORMS Inventor** hubs only — explicitly **not** the
Boost Move Hub (the City Hub is also excluded in current versions). The Pybricks
maintainers give two reasons:

- **Flash starvation** — the Move Hub has only **~106 KiB free flash** and is
  already using >95% of it; the feature does not fit alongside MicroPython.
  (City/Technic hubs have ~232 KiB free.)
- **"Different Bluetooth chip… not sure it has all the required functionality."**

Sources: pybricks discussions
[#1761](https://github.com/orgs/pybricks/discussions/1761),
[#218](https://github.com/orgs/pybricks/discussions/218); Xbox controller
[compatibility table](https://docs.pybricks.com/en/latest/iodevices/xboxcontroller.html).

### 2. Move Hub hardware

| Part | Detail |
|---|---|
| Main MCU | **STM32F070RB** — ARM Cortex-M0, **128 KB flash, 16 KB RAM** |
| Bluetooth | **ST BlueNRG-MS** — standalone BLE *network coprocessor*, connected over SPI, runs the full BLE stack on-chip and is driven by ACI/HCI commands |
| Flashing | Built-in **LEGO bootloader** (no SWD/hardware programmer needed — same path Pybricks uses) |

The newer hubs instead use a **TI CC2640** (Pybricks driver
`bluetooth_stm32_cc2640.c`). The Move Hub uses a different driver,
`bluetooth_stm32_bluenrg.c` — this is the "different Bluetooth chip" the
maintainers refer to.

### 3. The central-role concern is overturned

The BlueNRG-MS **supports the GAP central/master role**, and Pybricks' *own*
Move Hub driver already calls the central-role ACI functions:

```c
aci_gap_init_begin(GAP_PERIPHERAL_ROLE | GAP_CENTRAL_ROLE, ...);
aci_gap_start_general_conn_establish_proc_begin(ACTIVE_SCAN, ...);
aci_gap_create_connection_begin(...);
aci_gap_terminate_gap_procedure_begin(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC);
```

So **scanning for and connecting to a peripheral (the Xbox controller) is not
blocked by the hardware.** The dominant real blocker — flash space — is a
*Pybricks-specific* problem (MicroPython eats the budget), which a dedicated
firmware avoids.

Sources:
[`bluetooth_stm32_bluenrg.c`](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/drv/bluetooth/bluetooth_stm32_bluenrg.c),
[BlueNRG-MS ACI manual (UM1865)](https://www.st.com/resource/en/user_manual/um1865-the-bluenrgms-bluetooth-le-stack-application-command-interface-aci-stmicroelectronics.pdf).

### 4. Conclusion

Direct Xbox→Move Hub pairing is **plausible** with dedicated custom firmware.
The remaining unknowns to settle in a spike:

- **(a)** the BlueNRG-MS **GATT-client + pairing/bonding (LE security)** path
  needed to read an Xbox HID-over-GATT controller, and
- **(b)** fitting the whole firmware in **128 KB flash / 16 KB RAM**.

## Firmware options (platforms / languages / frameworks)

| Option | Language(s) | Reuse | Flash risk | Verdict |
|---|---|---|---|---|
| **A. Extend Pybricks** (`pybricks-micropython`) | C (pbio BlueNRG driver) + Python user code | Highest — BlueNRG driver, motor/LPF2 port drivers, bootloader integration all already exist | **High** — competes with MicroPython for ~106 KiB | Best **spike vehicle** (prove the BLE-central→Xbox path in-tree with max reuse); ship target only if it fits |
| **B. Bare-metal STM32 + ST BlueNRG-MS ACI middleware** (STM32Cube / X-CUBE-BLE1) | C | ST ships BlueNRG-MS master/central + GATT-client examples; the BLE stack lives on the BlueNRG so host-side code is small | **Low** — no MicroPython; comfortably fits 128 KB | **Recommended ship target** |
| **C. Zephyr RTOS** | C | STM32F0 port + `hci_spi` BlueNRG driver + full BLE central + HID-over-GATT (HOGP) host | Medium — RAM/flash budget on the F070 is tight | Cleanest BLE host APIs; viable but budget-risky on this MCU |
| **D. libopencm3 / minimal HAL + hand-rolled ACI** | C | Lowest — most from scratch | Lowest | Fallback only if B/C prove too heavy |

### Cross-cutting building blocks (needed regardless of option)

- **Flashing & recovery** — the Move Hub's built-in LEGO bootloader. Establish a
  reliable Pybricks ↔ custom ↔ stock recovery loop *first*, so a bad flash is
  never fatal.
- **Motor / port control** — the LEGO Power Functions 2 (LPF2) port protocol plus
  STM32 PWM for the two built-in track motors and the external head motor. Reuse
  the confirmed Vernie (17101) port map from the middleman project:
  **A/B = track motors, C = color/distance sensor, D = head motor**
  (see [`../../lego-boost-xbox/README.md`](../../lego-boost-xbox/README.md)).
- **Xbox HID** — a BLE **HID-over-GATT (HOGP)** client plus report parsing. The
  Xbox Series controller report byte layout is already known from the middleman's
  Gamepad-API code (`../../lego-boost-xbox/web/boost-xbox.js`) and the sibling
  [`xbox-switch-bridge`](../../xbox-switch-bridge/) (Bluepad32) package.
- **Reverse-engineering references** —
  [JorgePe/BOOSTreveng](https://github.com/JorgePe/BOOSTreveng),
  the [LEGO Wireless Protocol v3 spec](https://lego.github.io/lego-ble-wireless-protocol-docs/).

## Recommended approach

1. Use **Option A (Pybricks in-tree)** as the *spike vehicle* — it has the most
   reuse and the BlueNRG central calls already exist, so it is the fastest way to
   answer the BLE-central→Xbox question.
2. Use **Option B (bare-metal + BlueNRG-MS ACI)** as the *ship target* once the
   spike confirms feasibility — it sidesteps the flash blocker entirely.
3. **Keep the existing middleman** ([`lego-boost-xbox`](../../lego-boost-xbox/))
   as the supported, working fallback throughout.

## Feasibility spike

Ordered steps to de-risk before any full firmware build:

1. **Confirm the flashing/recovery loop** on the hub (Pybricks ↔ custom ↔ stock)
   so any flash is recoverable. (Pybricks flashing is already proven on this hub.)
2. **Prove BLE central → Xbox in-tree.** In a `pybricks-micropython` Move Hub
   build, exercise the BlueNRG driver's existing central calls to scan for and
   connect to the Xbox controller; verify GATT discovery and the pairing/bonding
   (LE security) handshake the Xbox HID service requires. **Highest-risk unknown.**
3. **Read one HID report.** Subscribe to the controller's HID input
   characteristic and log raw report bytes; map to sticks/buttons using the layout
   already in the middleman / `xbox-switch-bridge`.
4. **Drive one motor** from a hard-coded value via the LPF2 port/PWM path.
5. **Close the loop** (stick → motor) and **measure flash/RAM**. Decide A-vs-B
   for the ship firmware based on whether it fits beside MicroPython.

**Success criterion:** the Move Hub, running spike firmware, scans, pairs with,
and reads input from an Xbox Series controller with **no PC or second board in
the loop**, and moves a motor in response — observed live. Recovery to
stock/Pybricks firmware verified at the end.
