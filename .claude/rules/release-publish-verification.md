# The Release Pipeline Reported Success for Months While Publishing Nothing

`build-firmware.yml` is the only path by which firmware reaches a human or a
robot: the web flasher fetches from the Pages site it deploys, and OTA fetches
what it attaches to the release. Between 2026-07 and 2026-09-12 it published
**zero binaries for all 16 projects** on every run, and reported success each
time. Four separate mechanisms had to line up for that, and each is worth
knowing on its own.

## 1. An artifact's internal layout is the LCA of the uploaded paths

`actions/upload-artifact` roots an artifact at the **least common ancestor** of
the paths it is given. The build job uploads four paths, all under
`packages/<project_path>/build/`, so that prefix is stripped at upload time and
the downloaded artifact looks like:

```
kids-audio-toy.bin
bootloader/bootloader.bin
partition_table/partition-table.bin
```

The consumer job looked under `artifacts/firmware-<id>/packages/<path>/build/`
— a directory the artifact never contains. **The upload path and the download
path are not the same path**, and nothing in either job's YAML says so.

- **Check**, whenever a job consumes an artifact another job produced:
  `gh run download <run-id> -n <artifact> -D /tmp/x && find /tmp/x -type f`.
  Read the layout; do not derive it from the upload step's globs.
- The same rule explains a working case: a single-path upload roots at that
  file's own directory, so single-file artifacts appear to "just work" and
  teach the wrong lesson.

## 2. `|| echo "WARN: …"` converts a total failure into a green run

Every `cp` was written `cp "$src/$f" "$dest/" 2>/dev/null || echo "WARN: missing
$f for $project_id"`. All 48 copies failed on every run. The step exited 0, the
job went green, and the only evidence was a warning line inside a passing job
that nobody opens.

**A guard that downgrades a failure to a log line must be justified per call**,
not applied to a whole block. `ota_data_initial.bin` genuinely is optional —
only projects whose `partitions.csv` declares a `data,ota` row emit one — and
that single legitimate case is what made the pattern look reasonable for the
three files that are mandatory. Collect failures and exit non-zero at the end of
the loop, so one run names every broken project instead of only the first.

## 3. Nothing downstream can notice, because the manifest is derived from source

`tools/generate-flasher-manifests.sh` builds each `manifest.json` part list from
the project's `partitions.csv`, not from the files on disk. So it emitted
complete, correct-looking manifests for directories holding nothing, Pages
deployed them, and the release attached them. The site served a four-part
manifest for `robocar-unified` v0.2.2 in which every referenced path returned
404.

**A generator that reads a declaration rather than the artifact cannot detect a
missing artifact.** That is a reasonable design — offsets belong to
`partitions.csv` — but it means the *only* possible check is fetching what was
published. See §5.

## 4. `x-cache: HIT` on a 404, and a cache-buster that is not one

GitHub Pages serves through Fastly, which **does not include the query string in
its cache key**. Measured with a control: a brand-new path with `?v=111` returns
`x-cache: MISS`; the same path with `?v=222` returns `x-cache: HIT`; a second
brand-new path returns `MISS`. So `?v=${{ github.run_id }}` appended to a probe
URL changes nothing.

`curl --retry-all-errors` does not help either: curl exits 0 on a 404 without
`--fail`, so no retry engages (0.17 s, one request — against ~31 s for a
genuinely retried refused connection). Tracked in #542.

**Cache-bust a Pages URL by changing the path, not the query**, and treat retry
flags as covering connection-level transients only.

## 5. The only check that would have caught this is fetching the artifact

Not a linter, not a schema, not a green job — a `curl` against the published
URL. `build-firmware.yml` now does it in two places, and both must stay:

| Gate | Asserts |
|---|---|
| `Verify published firmware parts` | every `builds[].parts[].path` of every generated manifest returns 200 from the deployed Pages site, with a matching byte size |
| `Verify release assets` | every staged basename is present in `.assets[].name` **by name**, then each matched `browser_download_url` fetches 200 |

Two design points, both learned the hard way:

- **Compare names, never counts.** A count comparison is name-blind: green when
  one expected file is swapped for an unrelated one (same total), red on an
  unrelated pre-existing asset outside the naming scheme (different total). Both
  directions were reproduced before and after the fix.
- **The byte check is a size check.** A served file of identical length with
  different content passes at exit 0 — measured. Catching that needs a checksum;
  do not describe the gate as catching wrong bytes.

Neither gate is worth anything unverified. When changing either, run it against
a deliberately broken input and require red — the live site was itself such an
input while the bug was open, which is how the Pages gate was first shown to
work.

## 6. A run's conclusion is not a pipeline's exit code

`gh run watch <id> --exit-status | tail -40` reports **`tail`'s** status. A
failed run reads as `0`. This cost two wrong "it passed" readings in one session,
in a repo whose `build-guide-drift-guard.md` § 2 already documents the same trap
for the drift guard. Drop the pipe, or read `gh run view <id> --json conclusion`
— and prefer the latter regardless, since a watch cut short by a timeout also
exits non-zero without the run having failed.

## When it bites

- Any job consuming an artifact a different job uploaded — the layout question
  is invisible in both halves of the YAML.
- Any publish step whose failure mode is *absence*. A missing file produces no
  error anywhere upstream of the fetch.
- Reading a CI outcome through a pipe, a `tail`, or a `| grep`.

## Related

- [`web-flasher.md`](web-flasher.md) — `flasher.json` scopes the release/flasher
  set, `project-matrix.json` scopes CI; the same which-file-drives-which-pipeline
  distinction
- [`build-guide-drift-guard.md`](build-guide-drift-guard.md) § 2 — verify a
  guard by running it with a negative control; § "read the guard's own exit
  code, not a pipe's"
- `~/repos/.claude/rules/release-artifact-verification.md` — the portfolio-level
  law this is an instance of: a green pipeline is not a shipped artifact
- `~/.claude/rules/tool-use-patterns.md` — control-test any negative that gates
  an action
- Issues #537, #538 (the bug and its gate), #542 (the inert cache-buster), #544
  (the upload step, upstream `softprops/action-gh-release#836`)
