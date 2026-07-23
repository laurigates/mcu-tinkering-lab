# raw_usb — Component Design (Milestone 1)

The M1 component: a minimal USB device stack that relays primitives between
`usb_rpc` (host PC) and the ESP32-S3's DWC2 OTG-FS controller. This is the
firmware-side counterpart of `facedancer/backends/espdancer.py` (M3).

It complements `docs/architecture.md` (system view) and ADR-0006 (why we bypass
`usbd`). The authoritative host↔device RPC is `docs/rpc-protocol.md`.

## Layout

```
components/raw_usb/
├── CMakeLists.txt        # idf_component_register; PRIV_REQUIRES tinyusb_port
├── idf_component.yml     # (managed deps: none — the port is vendored)
├── include/raw_usb.h     # primitive surface + descriptor-table API + events
├── raw_usb.c             # dcd_event_handler + relay pump + primitives
├── endpoint_table.h      # EP0..6 state (single-writer-owned)
└── tinyusb_port/         # the DCD-only TinyUSB port (ADR-0006)
```

## Hardware facts (driving the design)

- Controller: Synopsys DWC2 OTG-FS, internal PHY, Full Speed only (12 Mbps).
- Register base `0x60080000`. **Endpoints: `DWC2_EP_MAX = 7` incl. EP0 → 6 non-control.**
- Interrupt: `dcd_dwc2.c` allocates it via IDF `esp_intr_alloc`.
- `dcd_dcache_*` are defined **inside** `dcd_dwc2.c` (no IDF cache → no DMA).

## States and lifecycle

```
RAW_USB_IDLE ──connect()──▶ RAW_USB_CONNECTED ──configure_ep()──▶ RAW_USB_ACTIVE
     ▲                              │                                        │
     └──── disconnect()/reset() ◀───┴──── bus_reset event ◀──────────────────┘
```

- `raw_usb_connect(dev_desc, cfg_desc, str_desc, speed, quirks)` stores the
  descriptor pointers in the **runtime descriptor table**, calls `dcd_init`
  then `dcd_connect` (assert D+ pull-up → the target host sees attach).
- Speed ≠ `DeviceSpeed.FULL` → log warning, downgrade (clone greatdancer).
- `quirks & MANUAL_SET_ADDRESS` → pump will *not* auto-call `dcd_set_address`
  on a SET_ADDRESS setup that the host acknowledges; the host drives it via
  `raw_usb_set_address`.

## Runtime descriptor table

TinyUSB's `usbd` normally serves GET_DESCRIPTOR from `tud_descriptor_*_cb`.
We bypass `usbd`, so GET_DESCRIPTOR arrives as `EVENT_SETUP` and the *host*
decides; but the DCD still needs device/config/string descriptor *pointers*
for `dcd_init`/enumeration plumbing. We keep them in RAM:

```c
typedef struct {
    const tusb_desc_device_t       *device;        // 18 B
    const uint8_t                  *config;        // 9 + Σintf
    const char *const              *strings;        // NULL-terminated
    uint8_t                         string_count;
} raw_usb_descriptors_t;
```

`raw_usb_connect` copies the incoming bytes (from the `CONNECT` payload) into a
static DMA-capable buffer (internal RAM, 4B-aligned) and points the table at
it. The pump serves GET_DESCRIPTOR by replying with the relevant table entry via
`dcd_edpt_xfer` on EP0 IN — but only when the *host* asks it to. Typically the
host's `USBDevice` standard-request handler sends the descriptor bytes itself
over `raw_usb_send(EP0_IN, …)`, then `raw_usb_ack_status`. The table is the
fallback / what `dcd_init` reads for `bMaxPacketSize0`.

## Endpoint table (single-writer, ADR-0005)

Owned exclusively by the pump task on core 1:

```c
typedef enum { EP_UNUSED=0, EP_OPEN, EP_STALLED } ep_state_t;
typedef struct {
    ep_state_t state;
    uint8_t    addr;        // w/ direction bit
    uint8_t    type;        // control/bulk/int/iso
    uint16_t   max_packet;
    // for OUT eps: the buffer the DCD fills; for IN: send buffer
    uint8_t    *buf; uint16_t buf_cap; uint16_t buf_len;
} ep_entry_t;
static ep_entry_t s_ep[DWC2_EP_MAX];   // index 0 = EP0
```

`raw_usb_configure_ep` validates count < `DWC2_EP_MAX`, fills an entry, and the
pump calls `dcd_edpt_open(&desc_ep)`. `dcd_edpt_close_all` on disconnect /
SET_CONFIG(0).

## Control-transfer state machine (the host drives it)

We expose every setup packet to the host and let facedancer's `USBDevice`
request handlers decide. The pump only coordinates the DCD plumbing:

```
EVENT_SETUP(ep0, setup[8]) ──▶ host
   host responds with ONE of:
     raw_usb_send(EP0_IN, data)      → DATA stage IN   (or descriptor reply)
     raw_usb_ack_status(OUT, EP0)    → host consumed the setup, ZLP OUT status
     raw_usb_ack_status(IN,  EP0)    → no-data control, ZLP IN status
     raw_usb_stall(EP0, dir)         → STALL the control endpoint
```

On `SET_ADDRESS` specifically: if `MANUAL_SET_ADDRESS` quirk is **clear**, the
pump calls `dcd_set_address` after the host ACKs the status stage (mirror
moondancer `QuirkFlag::MANUAL_SET_ADDRESS`). If **set**, the host calls
`raw_usb_set_address` explicitly and the pump does nothing automatic.

## dcd_event_handler (ours)

ISR-context only. No endpoint calls. Just enqueue:

```c
void dcd_event_handler(dcd_event_t const *ev, bool in_isr) {
    xQueueSendFromISR(s_event_q, ev, &hpw);
}
```

The pump drains `s_event_q` and translates:

| DCD event | → `raw_usb` action | → `usb_rpc_send_event` |
|-----------|--------------------|------------------------|
| `DCD_EVENT_BUS_RESET` | clear endpoint table, drop pull-up logic | `EVENT_BUS_RESET` |
| `DCD_EVENT_SETUP_RECEIVED` | (no DCD call) | `EVENT_SETUP(ep=0, setup[8])` |
| `DCD_EVENT_XFER_COMPLETE` ep IN | mark IN done; allow next `raw_usb_send` | `EVENT_SEND_COMPLETE(ep)` |
| `DCD_EVENT_XFER_COMPLETE` ep OUT| copy `xfer_complete.len` bytes from the ep's OUT buffer | `EVENT_OUT_PACKET(ep, len, data)` |
| `DCD_EVENT_SUSPEND` / `RESUME` | (informational; pass through) | (optional, M4) |

NAK: DWC2 has no explicit NAK event in the TinyUSB DCD model; IN-NAK is inferred
when `raw_usb_send` has no pending data and the host polls. We surface NAK via
`EVENT_NAK` only if needed (deferred to M2/M4; hydradancer emits it, moondancer
derives it). Documented in CHECKLIST.

## Primitive surface (header)

```c
esp_err_t raw_usb_init(void);                          // once at boot
esp_err_t raw_usb_connect(const raw_usb_descriptors_t *d,
                          uint8_t ep0_max, uint8_t speed, uint8_t quirks);
esp_err_t raw_usb_disconnect(void);
esp_err_t raw_usb_reset(void);
esp_err_t raw_usb_configure_ep(uint8_t addr, uint8_t type, uint16_t max_pkt);
esp_err_t raw_usb_send(uint8_t ep, const uint8_t *data, uint16_t len);
esp_err_t raw_usb_stall(uint8_t ep, uint8_t dir);
esp_err_t raw_usb_clear_halt(uint8_t ep, uint8_t dir);
esp_err_t raw_usb_ack_status(uint8_t dir, uint8_t ep);
esp_err_t raw_usb_set_address(uint8_t addr, uint8_t defer);
ep_state_t raw_usb_ep_state(uint8_t ep, uint8_t dir);  // for dispatcher sanity
bool       raw_usb_ready(void);                        // enumerated?
```

`usb_rpc`'s dispatch table maps each RPC message (see `docs/rpc-protocol.md`)
1:1 to these. In M1, before M2 wires the dispatcher, a tiny in-firmware test
hook can call `raw_usb_connect` with a built-in trivial HID descriptor to prove
enumeration.

## M1 sub-tasks (in order)

1. **Vendor the port** — copy `dcd_dwc2.c` + `dwc2_common.c` + the dwc2_*.h +
   `tusb_fifo.c` + headers into `tinyusb_port/`; write `tusb_config.h`
   (OPT_MCU_ESP32S3, no host, no classes, `osal_none`); write the port's
   `CMakeLists.txt`. Carry `LICENSE_TINYUSB.txt`.
2. **Dry compile** — fix includes/`usbd_pvt.h`/OSAL stubw; remove
   `esp_tinyusb` from `main/idf_component.yml`.
3. **`raw_usb_init` + `dcd_event_handler`** — install ISR, empty event queue,
   confirm `DCD_EVENT_BUS_RESET` fires on a host cable plug.
4. **Runtime descriptor table + `raw_usb_connect`** — `dcd_init` + `dcd_connect`;
   verify the S3 enumerates as a built-in trivial HID (host PC sees it).
5. **`raw_usb_configure_ep` + `raw_usb_send`** — round-trip a HID report to the
   host PC.
6. **Control transfer stages** — `ack_status`, `stall`, `set_address` quirk.
7. **Event → `usb_rpc_send_event`** wiring (full M2 lands the backpressure).

M1 exit gate: a built-in trivial-HID descriptor lets the S3 enumerate on a
Linux/macOS host and a `raw_usb_send` pumps an input report the host reads.