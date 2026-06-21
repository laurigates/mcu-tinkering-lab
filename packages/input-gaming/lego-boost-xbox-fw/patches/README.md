# Patches against pybricks-micropython

Patches here are applied to the `external/pybricks-micropython` checkout with
`just boost-fw::enable-xbox` (i.e. `git -C external/... apply patches/<name>.patch`).

## `enable-xbox-movehub.patch` (produced during Phase 3)

This patch does not exist yet — it is the output of the Phase-3 investigation,
which needs the cloned source (`just boost-fw::setup`). Goal: turn on the
`XboxController` feature for the Move Hub build that is otherwise excluded.

### Where to look (in `external/pybricks-micropython`)

- **Per-hub feature/capability flags** — `bricks/movehub/` (its `mpconfigport.h`
  / `*.mk` / config) and the shared `lib/pbio/include/pbio/config.h` +
  per-platform `pbio_config` that gate optional features by hub.
- **XboxController module registration** — where the `pybricks.iodevices`
  `XboxController` class is conditionally compiled (grep the `bricks/` and
  `pybricks/` module tables and the `cc2640` vs `bluenrg` driver guards).
- **Pairing path** — `lib/pbio/drv/bluetooth/bluetooth_stm32_bluenrg.c` vs
  `bluetooth_stm32_cc2640.c`; the generic flag is
  `PBDRV_BLUETOOTH_PERIPHERAL_OPTIONS_PAIR` in `lib/pbio/include/pbdrv/bluetooth.h`.

### How to capture the patch

After making the minimal edits in the checkout:

```sh
git -C external/pybricks-micropython diff > patches/enable-xbox-movehub.patch
```

Then rebuild with `just boost-fw::enable-xbox && just boost-fw::build` and record
the result (linker overflow vs fits) in `../docs/spike-log.md`.
