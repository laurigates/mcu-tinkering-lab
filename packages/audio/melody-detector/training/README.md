# melody-train

Training pipeline for the melody-detector ROI classifier. Synthetic
data → PyTorch CNN → ONNX → ESP-PPQ INT8 → `.espdl` artifact embedded
in the firmware. All-OSS, fully reproducible from a seed.

See [PRD-011](../../../../docs/requirements/PRD-011-melody-detector.md),
[ADR-017](../../../../docs/decisions/ADR-017-melody-detector-esp-dl.md),
and the [ESP-DL deployment reference](../../../../docs/reference/esp-dl-deployment.md).

## Status

**B1 — synthetic dataset generator** (this slice). Tracks B2–B4 follow.

| Slice | Status | Module |
|---|---|---|
| B1 — `gen.py` synthetic ROI generator | this slice | `melody_train.gen` |
| B2 — `model.py` + `dataset.py` + `train.py` | not started | `melody_train.{model,dataset,train}` |
| B3 — `export.py` ONNX export | not started | `melody_train.export` |
| B4 — `quantize.py` ESP-PPQ INT8 quantization | not started | `melody_train.quantize` |

## Setup

```bash
just sync           # base deps (numpy, pillow, albumentations, click)
just sync-train     # adds torch + torchvision (B2)
just sync-quantize  # adds esp-ppq + onnx + onnxruntime (B4)
```

The project uses [`uv`](https://docs.astral.sh/uv/) and pins Python
3.11 via `.python-version`.

## Generate a dataset (B1)

```bash
# Default: 5000 samples per class, seed 0
just gen

# Smoke test
just gen 0 /tmp/melody-mini 100

# Direct invocation
uv run python -m melody_train.gen --seed 0 --out data/ --per-class 5000
```

Output structure:

```
data/
├── empty/000000.png ... 004999.png
├── quarter/...
├── half/...
├── rest/...
├── other/...
└── manifest.json
```

The `manifest.json` records seed, per-class count, image size, schema
version, and class list. Two runs with the same seed produce
byte-identical PNG files for every class/index — this is the
reproducibility gate enforced in tests.

## Class semantics

| Class | Visual | Audio behaviour |
|---|---|---|
| `empty` | clean cell of paper | no note (skipped by CV pre-filter) |
| `quarter` | filled notehead | short note at the row's pitch |
| `half` | hollow notehead | long note at the row's pitch |
| `rest` | hand-drawn squiggle | hold the beat without sound |
| `other` | smudge / fingerprint / partial drawing | **silent** (noise rejection) |

The `other` class is a noise-rejection bucket — not a fallback for
ambiguous noteheads. Augmentation variants of legitimate noteheads
stay in their own class so the model learns to recognize messy real
drawings, while genuine non-note ink (smudges, paper artifacts) is
silently dropped.

## Quality gates

```bash
just test           # pytest
just test-cov       # with coverage report
just lint           # ruff check
just format-check   # ruff format --check
just type-check     # ty check
just check          # all of the above
```

## Reproducibility

The pipeline is deterministic from `--seed`. Each class gets its own
RNG stream spawned from the top-level seed via `numpy.random.SeedSequence`,
so adding a class later doesn't perturb the existing classes.

The `numpy.random.Generator` is passed explicitly through every
function — there is no use of `numpy.random` global state. Tests verify
this by setting global seed to a value and confirming output doesn't
change.
