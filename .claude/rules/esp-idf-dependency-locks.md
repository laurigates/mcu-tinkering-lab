# Tracked ESP-IDF `dependencies.lock` Files and How They Get Updated

The root `.gitignore` ignores `dependencies.lock`, so most ESP-IDF projects here
resolve their `idf_component.yml` ranges fresh on every build. A few projects
track their lock instead (added with `git add -f`), which pins every managed
component to the version it was first resolved to. **Nothing moves a tracked
lock on its own.** Renovate has no ESP-IDF component-manager manager, and the
component manager only re-solves when the manifest changes. Without a refresh,
a pinned `espressif/mdns` misses the fixes to a parser that reads packets from
any host on the LAN (issue #643).

List the tracked set from git, not from this file:

```sh
python3 tools/refresh-idf-locks.py --list
```

## The update path

| How | When |
|---|---|
| `.github/workflows/refresh-idf-locks.yml` | 05:17 UTC on the 1st of each month, or `gh workflow run refresh-idf-locks.yml`. Opens or updates one PR on branch `chore/refresh-idf-locks` |
| `just refresh-idf-locks [packages/<domain>/<project> ...]` | Locally, all tracked locks or the ones named |

Both run `tools/refresh-idf-locks.py`, which runs `idf.py update-dependencies`
in the ESP-IDF container for each project. That command deletes the lock and
reconfigures, so the component manager re-resolves the manifest ranges and
writes a new lock with a new `manifest_hash`. The script adds three things on
top:

- **The target comes from the lock's own `target:` line.** It is passed as
  both `-DIDF_TARGET` and `IDF_TARGET`, because idf.py rejects the two
  disagreeing and `esp-idf-ci-action` exports `IDF_TARGET=esp32` for the whole
  job. Without this, an `esp32s3` project would be re-resolved for `esp32`.
- **The build directory and sdkconfig go to a temp dir.** A local `build/` or
  `sdkconfig` is left alone. `managed_components/` and a stub
  `main/credentials.h` do appear in the project, and both are gitignored.
- **A lock with only the `idf` entry is skipped**, and the skip is printed.
  There is nothing to re-resolve, and the idf version follows the container
  image. `gamepad-synth` is the current case. It also needs the bluepad32
  checkout to configure at all. If it ever gains a registry dependency, the
  refresh fails loudly until the workflow fetches bluepad32 too.

A project that fails gets its original lock back. The others still refresh,
and the script exits 1 naming each failure. The workflow then still opens the
PR for the projects that succeeded and ends red. It refuses to open a PR if
anything other than a tracked lock changed.

The PR is opened with the release-please App token, not `GITHUB_TOKEN`. A PR
opened with `GITHUB_TOKEN` does not trigger `pull_request` workflows, so
`build.yml` would never build it.

## Reviewing a lock-refresh PR

- **`build.yml` builds only projects listed in `.github/project-matrix.json`.**
  A tracked lock in a project with no matrix entry gets no gate.
  `camera-vision/gemini-vision` is in that state.
- **A green build does not prove the board boots.** A component bump can move
  the I2C driver generation and abort at the first boot with a clean build
  (`esp-idf-i2c-driver-generation-conflict.md`). For a change to
  `esp32-camera` (its SCCB layer) or any I2C-touching component, run that
  rule's check. A sensor-facing change also needs a bench boot.
- The refresh stays inside the manifest ranges. A major bump is a manual
  `idf_component.yml` edit followed by a refresh.

## Starting or stopping tracking

- Track: build the project once so a lock exists, then `git add -f
  packages/<domain>/<project>/dependencies.lock`. The next refresh picks it up
  with no further change, and the CI cache key in `_ci-build-esp32.yml` already
  hashes the lock.
- Stop: `git rm --cached` the lock. The root ignore keeps it out afterwards.

## Verify a change to the script or workflow

```sh
python3 -m unittest tools/test_refresh_idf_locks.py
just refresh-idf-locks packages/robocar/camera
```

The unit tests run the shipped script against a stub `idf.py`. The second
command does a real resolve against the component registry. Discard its lock
change afterwards if you only wanted the check. The workflow can only be
exercised from `main` (`gh workflow run refresh-idf-locks.yml`), so a change to
it is unverified until that dispatch has run.

## Related

- `esp-idf-sdkconfig.md`: the generated-config staleness trap. Same family of
  generated file whose inputs moved
- `esp-idf-i2c-driver-generation-conflict.md`: why a green lock bump can still
  abort at boot
- `containerized-builds.md`: the container the refresh runs in
