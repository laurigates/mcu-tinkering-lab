# Lint Python Through pre-commit, Not Bare `ruff`

The repo's ruff is the one pre-commit pins (`ruff-pre-commit` rev in
`.pre-commit-config.yaml`, with `ruff.toml`). A `ruff` on PATH can be another
version with other defaults: on the pan-tilt PR, ruff 0.16.9 passed files that
pre-commit's v0.15.6 failed on E402 and E731, and its `--unsafe-fixes` removed
`# noqa: E402` comments the pinned version needs. CI then failed on the
formatting of a file that had passed locally.

```sh
pre-commit run --files <changed files>   # what CI runs; repeat until clean
```

- **Run it twice.** The first pass fixes and fails (`--exit-non-zero-on-fix`),
  the second must pass. Stage the hook's changes in between.
- **A script that edits `sys.path` before importing** needs `# noqa: E402` on
  each import after it. Keep them; do not let a newer ruff's fixer strip them.
- **Agent-written Python** (workflow designers, generated tools) gets the same
  pass before it is committed. Five design scripts failed on these two rules.
