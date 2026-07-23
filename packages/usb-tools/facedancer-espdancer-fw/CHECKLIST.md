# espdancer — Implementation Checklist

Working backlog for the ESP32-S3 facedancer backend. Each item has an owner
milestone. Strikethrough (`~~`) = done. Source provenance tagged
`[xbox-switch-bridge]` = lifted/adapted from that repo; `[new]` = net-new;
`[facedancer]` = lands in the upstream facedancer clone.

## Milestone 0 — Scaffold (this commit)

> **Status:** all files written; **build not yet verified end-to-end** (the
> `mcu-tinkering-lab/esp-idf:v5.4` docker image wasn't built in the scaffolding
> session). `usb_rpc.c` passed a host `clang -fsyntax-only` check against ESP
> header stubs (framing + CRC + dispatch + rpc_task all parse clean). First real
> task: `rm -f sdkconfig && just build` and clear any remaining Kconfig/link
> errors before starting Milestone 1.

- [x] Project skeleton under `packages/usb-tools/facedancer-espdancer-fw` `[new]`
- [x] `CMakeLists.txt`, `Makefile`, `justfile`, `partitions.csv`, `flasher.json`,
      `version.txt`, `.gitignore` `[xbox-switch-bridge]`
- [x] `sdkconfig.defaults` (BLE dropped, WiFi coexistence knobs kept) `[xbox-switch-bridge]`
- [x] Build variants: `sdkconfig.debug-uart`, `sdkconfig.debug-jtag`,
      `sdkconfig.debug-usb` `[xbox-switch-bridge]`
- [x] `components/status_led/` — relabeled modes for emulator states `[xbox-switch-bridge]`
- [x] `components/log_udp/` — SoftAP + UDP log sink (SSID `espdancer-log`) `[xbox-switch-bridge]`
- [x] `components/usb_rpc/` — length-prefixed framing header + TCP server stub `[new]`
- [x] `main/main.c` — bring-up: status LED, (debug-usb) WiFi log, rpc transport, idle `[new]`
- [x] ADRs 0001–0005 + `docs/rpc-protocol.md` (the contract the firmware implements) `[new]`
- [x] `CLAUDE.md`, `README.md` `[xbox-switch-bridge]`

**Exit gate:** `just build` produces a flashable binary that boots, lights the
LED, and (debug-usb) advertises `espdancer-log` + accepts a TCP connection and
echoes the `HELLO` handshake. No USB emulation yet.

## Milestone 1 — Raw USB relay (the hard part)

> **Design:** documented in `docs/architecture.md`, `docs/raw-usb-design.md`,
> ADR-0006. ADR-0002's open question is **resolved**: we bypass TinyUSB's `usbd`,
> vendor a DCD-only TinyUSB port (`components/raw_usb/tinyusb_port/`), and
> provide our own `dcd_event_handler`. Investigation of `espressif__tinyusb`
> 0.19 confirmed: `usbd.c` strongly defines `dcd_event_handler` and its static
> `process_control_request` auto-handles all USB standard requests (an
> override-free blocker for Facedancer's host-driven model). Also surfaced a
> hard limit: **`DWC2_EP_MAX = 7` incl. EP0 → 6 non-control endpoints**.

- [x] Design docs: `docs/architecture.md`, `docs/raw-usb-design.md`, ADR-0006
      (resolves ADR-0002 open question) `[new]`

### Vendoring the port (ADR-0006)

- [x] Create `components/raw_usb/tinyusb_port/` skeleton + `LICENSE_TINYUSB.txt`
      (MIT, carried the notice) `[new]`
- [x] Write `tinyusb_port/tusb_config.h`: `OPT_MCU_ESP32S3`, no host, no
      classes, `osal_none` — and `tinyusb_port/CMakeLists.txt` with the bounded
      `srcs` set (dcd_dwc2.c + dwc2_common.c + tusb_fifo.c + headers) `[new]`
- [x] Copy vendored files from `espressif__tinyusb` v0.19.0 (preserving the
      `src/portable/synopsys/dwc2/`, `src/common/`, `src/device/`, `src/osal/`
      tree): dcd_dwc2.c, dwc2_common.c, dwc2_*.h (10), tusb_fifo.c/h,
      tusb_types/compiler/verify/debug/private/common.h, tusb_mcu.h, dcd.h,
      usbd_pvt.h, osal.h, osal_none.h, tusb_option.h — pinned to 0.19.0 `[new]`
- [x] `raw_usb.c` flips `RAW_USB_DCD_VENDORED=1`; uses real `device/dcd.h`,
      `common/tusb_types.h`, `tusb_option.h`; calls `dcd_init(rhport, &rh_init)`
      with `tusb_rhport_init_t{role,speed}` `[new]`
- [x] `components/raw_usb/CMakeLists.txt` `PRIV_REQUIRES tinyusb_port` (include
      path + dcd_*) `[new]`
- [ ] Dry compile: `rm -f sdkconfig && just build` against
      `mcu-tinkering-lab/esp-idf:v5.4`; fix residual `usbd_pvt.h`/OSAL stub /
      IDF `priv_requires` issues — **pending: docker image not yet built** `[new]`
- [ ] Remove `espressif/esp_tinyusb` dep from `main/idf_component.yml` (verify
      it's no longer needed for any KCONFIG_TINYUSB_* symbol the build variants
      reference) `[new]`

### raw_usb component

- [x] `components/raw_usb/include/raw_usb.h` — primitive surface + descriptor
      table API + event enum + quirks (mirrors `docs/raw-usb-design.md`) `[new]`
- [x] `components/raw_usb/raw_usb.c` — own `dcd_event_handler` (ISR enqueue à la
      ADR-0005), relay pump task, endpoint table (×7), descriptor table; DCD
      calls stubbed behind `RAW_USB_DCD_VENDORED=0` so it compiles & links today `[new]`
- [x] `components/raw_usb/tinyusb_port/` — vendoring manifest (`README.md`),
      `tusb_config.h`, and an empty-component `CMakeLists.txt` ready to swap in
      the three DCD source lines once files are copied `[new]`
- [x] `main.c` wired to `raw_usb_init()` + `raw_usb_pump_task` on core 1 `[new]`
- [x] Stub-compile check of `raw_usb.c` + `main.c` (host `clang -fsyntax-only`) `[new]`
- [ ] `raw_usb_init` + ISR install → demonstrate `DCD_EVENT_BUS_RESET` fires on
      a host cable-plug (no enumeration yet) `[new]`
- [ ] Runtime descriptor table + `raw_usb_connect` (`dcd_init` + `dcd_connect`) →
      prove the S3 enumerates on a Linux/macOS host as a built-in trivial HID `[new]`
- [ ] `raw_usb_configure_ep` (`dcd_edpt_open`) + `raw_usb_send` (`dcd_edpt_xfer`) →
      round-trip a HID input report the host PC reads `[new]`
- [ ] Control transfer stages: `ack_status` (ZLP), `stall`/`clear_halt`,
      `set_address` (+ `manual_set_address` quirk; mirror moondancer
      `QuirkFlag`) `[facedancer]`
- [ ] Speed handling: advertise `DeviceSpeed.FULL` only; warn + downgrade
      `HIGH`/`LOW` (clone greatdancer's wording) `[facedancer]`
- [ ] Bus reset + suspend/resume event capture (-> EVENT_BUS_RESET; suspend/
      resume deferred to M4) `[new]`

**Exit gate:** with a *built-in* trivial HID descriptor driven in-firmware
(not yet host-supplied), the S3 enumerates on a Linux/macOS host and a
`raw_usb_send` pumps an input report the host reads. Host-supplied descriptors +
full EVENT_→RPC wiring land in M2.

## Milestone 2 — RPC wiring (host↔device)

- [ ] `components/usb_rpc/usb_rpc.c` — dispatch table mapping every
      `FacedancerBackend` primitive to a `raw_usb_*` call and an `EVENT_*`
      reply (see `docs/rpc-protocol.md`) `[new]`
- [ ] Transport: extend `log_udp` lessons → real bidirectional TCP server on the
      SoftAP (port 4444), framed per `docs/rpc-protocol.md` `[new]`
- [ ] Backpressure: outbound event queue with watermark; `SEND` acknowledgment
      gated on `dcd` completion, not fire-and-forget `[new]`
- [ ] CRC8 + resync-on-magic; malformed-frame recovery `[new]`

**Exit gate:** `BACKEND=espdancer python examples/minimal.py` runs against an
S3 over the SoftAP and enumerates on the target host.

## Milestone 3 — Python backend `[facedancer]`

Lands in `external/facedancer/facedancer/backends/espdancer.py` (upstream clone).

- [ ] `espdancer.py` — subclass `FacedancerApp, FacedancerBackend`; clone
      `moondancer.py` / `hydradancer.py` shape
- [ ] `appropriate_for_environment()` — `BACKEND=espdancer` + reachable TCP host
- [ ] `connect` / `configured` / `send_on_endpoint` / `read_from_endpoint` /
      `stall_endpoint` / `clear_halt` / `ack_status_stage` / `service_irqs`
      implemented as framed RPC
- [ ] `service_irqs()` dequeues `EVENT_*` and forwards to
      `USBDevice.create_request / handle_data_available / handle_nak`
      (mirror moondancer `InterruptEvent` codes 10–13)
- [ ] Register in `facedancer/backends/__init__.py:__all__`
- [ ] Add to facedancer `README.md` supported-boards list

**Exit gate:** `examples/rubber-ducky.py` and `examples/mass-storage.py` run
over the S3 at Full Speed.

## Milestone 4 — Hardening + tests

- [ ] Endpoint count/type limits enforced up-front with a clear error (mirror
      the MAX3421 caveat in facedancer README) `[facedancer]`
- [ ] Unit tests for the rpc framing layer (`components/usb_rpc/test/`) `[new]`
- [ ] Integration smoke: enumerate as HID, CDC, then a small composite
      descriptor; verify on Linux + macOS targets `[new]`
- [ ] Throughput characterization over SoftAP; decide if a bulk batching scheme
      is needed (clone `HydradancerEvent` IN/OUT buffer-available semantics)
      `[facedancer]`
- [ ] Document the single-OTG-port limitation: **no USBProxy/MITM**
      (same as MAX3421 boards), Full-Speed only without a ULPI PHY board
      `[facedancer]`

## Out of scope (captured so we don't re-litigate)

- High-Speed emulation (needs custom ULPI PHY hardware — no common devkit has it)
- USBProxy / Medler-In-The-Middle (single OTG port; needs two USB ports)
- On-device MicroPython/CircuitPython (breaks the Facedancer host-library model)