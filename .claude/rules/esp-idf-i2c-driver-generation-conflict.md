# ESP-IDF Cannot Link the Legacy and New I2C Drivers Into One Binary — Ever

ESP-IDF 5.x ships two I2C driver generations: the legacy `driver/i2c.h`
(`i2c_driver_install`, `i2c_param_config`) and the new `driver/i2c_master.h`
/ `driver/i2c_slave.h` ("`driver_ng`", `i2c_new_master_bus`,
`i2c_master_bus_add_device`). **A single firmware image may link only one of
them, project-wide.** This is not a per-port runtime conflict — it is a
whole-binary link-time constraint, enforced by a `__attribute__((constructor))`
in the legacy driver (`components/driver/i2c/i2c.c`) that runs before
`app_main` and aborts if the new driver's `i2c_acquire_bus_handle` symbol is
also linked in, regardless of which I2C port either driver actually uses:

```
E (731) i2c: CONFLICT! driver_ng is not allowed to be used with this old driver
abort() was called at PC 0x...
```

**The build succeeds either way.** This is a link-compatible, boot-time-only
failure — `idf.py build` / `just build` report green because nothing at
compile or link time objects to both symbol sets coexisting. The abort only
fires the first time the constructor runs, i.e. on the very next boot. A
clean CI build is not evidence this is fine.

## Why it recurs: a dependency can flip generation under you, silently

Neither driver generation is usually chosen by your own code — it is chosen
by whichever managed component talks I2C, via that component's own Kconfig
default, and that default **changes across versions with no compile error to
flag it**:

- `esp-idf-lib/i2cdev` moved from the legacy driver to `i2c_new_master_bus()`
  at 2.0.0 (in use here for the TCA9548A/PCA9685/MCP23017 stack, `i2c_bus.c`).
- `espressif/esp32-camera`'s SCCB layer ships **both** implementations
  (`sccb.c` legacy, `sccb-ng.c` new) behind a `choice
  SCCB_HARDWARE_I2C_DRIVER_SELECTION` Kconfig, and its own upstream default
  has itself changed generation across releases.

Robocar-unified hit this twice. The first time, `sdkconfig.defaults` was
hand-pinned to `CONFIG_SCCB_HARDWARE_I2C_DRIVER_LEGACY=y` with a comment
recording *why* — to match i2cdev, which was legacy at the time. Nobody
revisited that pin when i2cdev was later bumped to 2.1.2 (new driver by
default), so the two components silently diverged and the project
reintroduced the exact abort the pin was written to prevent. **A config
comment that hardcodes "component X must use driver generation Y because
component Z does" is a claim about Z's *current* version — it has no
mechanism to notice when Z's default moves out from under it.**

## The check

Before or after bumping the version of *any* component that touches I2C
(managed or vendored), confirm every I2C-touching component agrees on
generation. Per component, read the Kconfig choice or grep its source for
which API it calls — don't infer generation from memory or from what a past
comment says another component "is":

```sh
grep -rl "i2c_new_master_bus\|i2c_master_bus_add_device" managed_components/*/  # new-driver callers
grep -rl "i2c_driver_install\|i2c_param_config" managed_components/*/            # legacy-driver callers
```

If both lists are non-empty, at least one component has a Kconfig choice that
must be flipped to match the other — check each hit's own Kconfig for a
`*_I2C_DRIVER_*` / `*_USE_LEGACY_I2C*`-shaped option before assuming it can't
be changed. For components that vendor **both** implementations behind a
build-time choice (like esp32-camera's `sccb.c` / `sccb-ng.c`), grepping the
source isn't sufficient by itself — check which file the Kconfig selection
actually compiles in (`git ls-files` / `CMakeLists.txt` conditionals), or just
read the resolved `sdkconfig` for the `*_DRIVER_LEGACY` / `*_DRIVER_NEW` line
after a build.

After changing any `*_I2C_DRIVER_*`-shaped Kconfig symbol in
`sdkconfig.defaults`, follow `esp-idf-sdkconfig.md`: delete the generated
`sdkconfig`, `just clean`, rebuild, and grep the regenerated `sdkconfig` for
the symbol to confirm it took (a stale `sdkconfig` silently keeps the old
value even with `sdkconfig.defaults` corrected).

## When it bites

- Bumping any managed component's version pin (`idf_component.yml`) where the
  component talks I2C, however indirectly — this is exactly what moved
  i2cdev off the legacy driver with no accompanying error anywhere in the
  dependency's own changelog wording.
- Adding a **second** I2C-touching managed component to a project that
  already has one settled on a generation (e.g. adding `esp32-camera` to a
  project already using `i2cdev`, or vice versa) — the new component's
  Kconfig default may not match.
- Any `sdkconfig.defaults` comment that names another component's driver
  choice as its justification. Treat it as a pin that needs re-verifying
  every time that other component's version changes, not a fact.

## Related

- `esp-idf-sdkconfig.md` — the generated `sdkconfig` staleness trap this
  fix's verification step depends on; a corrected `sdkconfig.defaults` alone
  proves nothing until the generated file is regenerated.
- `~/.claude/rules/diagnose-at-the-failure-point.md` — the abort's own
  message names the mechanism (`driver_ng` vs "this old driver") precisely;
  the fix was reading that constructor's source, not guessing from the
  backtrace addresses.
- `gated-init-paths.md` / `esp-idf-rmt-rx.md` / `freertos-task-gotchas.md` —
  sibling ESP-IDF traps where a build that succeeds says nothing about
  whether the firmware boots.
