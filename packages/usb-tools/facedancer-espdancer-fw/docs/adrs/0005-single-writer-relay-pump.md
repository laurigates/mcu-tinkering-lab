# ADR-0005: Single-Writer Relay Pump (No Mutex)

**Status**: Accepted
**Date**: 2026-07-22
**Confidence**: High

## Context

TinyUSB callbacks run at higher priority than the application task. Calling
endpoint-send primitives from inside a callback races the application's own
send path and corrupts endpoint state — switch-usb-proxy documented this
exactly: *"Never call tud_hid_report() from the callback — the TinyUSB task
can preempt the report task mid-send, causing concurrent calls and corrupted
state (was the root cause of Switch error 2162-0002)."* It solved it with a
FreeRTOS queue + single-writer pattern (ADRs credit xbox-switch-bridge
ADR-0005).

## Decision

Use a **single FreeRTOS queue + single-writer pump** for the USB data path:

- TinyUSB DCD callbacks (priority 5) **only enqueue**: events go to the
  device→host event queue (`OUT_PACKET`, `SETUP`, `SEND_COMPLETE`, `NAK`);
  incoming OUT data is copied into the queue, never sent in the callback.
- A single **relay pump task** (priority 4) is the **sole writer** to USB
  endpoints: it drains the host→device command queue and calls
  `dcd_edpt_xfer`, and it's the sole reader of the device→host event queue
  that frames them into `usb_rpc_send_event()`.
- `s_timer_counter`-style single-writer state: a counter the pump owns
  exclusively needs no synchronization.

Endpoint staging buffers (IN ready / OUT pending) live in the pump task; the
control channel (`usb_rpc`) is the only producer into its command queue.

## Consequences

- No mutexes on the hot path; contention is none, correctness is by
  construction. Same discipline that let xbox-switch-bridge's 125 Hz loop be
  lock-free.
- The pump runs on core 1, the control-channel task on core 0 (mirroring
  xbox-switch-bridge's core split: USB on core 1, comms on core 0).
- Bound the queue depth; a full queue drops with a logged warning
  (`queue_reply` in switch-usb-proxy does exactly this).

## Alternatives Considered

- **Mutex on every endpoint call.** Works, but TinyUSB callback priority makes
  it deadlock-prone and slower than the queue pattern.

## Reference

- `switch-usb-proxy/main/main.c` — `queue_reply` / `input_report_task`
- `xbox-switch-bridge/docs/adrs/0005-single-writer-shared-state.md`