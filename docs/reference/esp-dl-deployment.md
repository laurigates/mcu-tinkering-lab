# ESP-DL Deployment Reference

Concrete patterns for deploying an INT8 CNN onto an ESP32-S3 via ESP-DL
v3.x and ESP-PPQ. Distilled from the upstream Espressif docs and source
on 2026-05-04. Use this as the operational complement to
[ADR-017](../decisions/ADR-017-melody-detector-esp-dl.md) and the
high-level [TinyML frameworks reference](tinyml-esp32-frameworks.md).

Confirmed against ESP-DL **v3.3.2** (latest on the IDF Component
Registry). The README on GitHub still says 3.2.0 but the registry is
ahead — pin against the registry version.

## Compatibility matrix

| Component | Required version |
|---|---|
| `espressif/esp-dl` | `>=3.3.2` |
| ESP-IDF | `>=5.3` (5.4 confirmed; 6.0 currently broken — see issue #295) |
| ESP-PPQ | latest from PyPI; ships with PPQ fork |
| PyTorch | 2.x; CPU build is sufficient for quantization calibration |
| Target SoC | ESP32-S3 (PIE SIMD), ESP32-P4 (per-channel + AI accelerator), ESP32 (no SIMD, slow) |

`.espdl` schema changed at 3.1.0 — models quantized for older runtimes
are not compatible. Always re-export when bumping ESP-DL.

## Manifest

```yaml
# main/idf_component.yml
dependencies:
  idf:
    version: '>=5.3'
  espressif/esp-dl: '>=3.3.2'
```

## Loading a model — EMBED_FILES path (recommended for v1)

The model is linked into the app binary as a rodata blob. Lower
friction than SPIFFS for development; switch to a SPIFFS partition
later if OTA-updatable models without firmware reflash become a
requirement.

### CMakeLists.txt

```cmake
# Place BEFORE idf_component_register():
set(embed_files "${CMAKE_CURRENT_SOURCE_DIR}/../model/model.espdl")

idf_component_register(
    SRCS "app_main.cpp" "inference.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp-dl
    EMBED_FILES ${embed_files}
)

# Place AFTER idf_component_register() — REQUIRED.
target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)
```

> **Footgun**: plain `EMBED_FILES` without `target_add_aligned_binary_data`
> silently produces misaligned data. FlatBuffers zero-copy
> deserialization needs 4-byte alignment; misalignment doesn't crash —
> it corrupts inference results. Always use the aligned wrapper.

### Inference (C++ only — no C API)

```cpp
#include "dl_model_base.hpp"
#include "fbs_loader.hpp"
#include "dl_tensor_base.hpp"

extern const uint8_t model_espdl_start[] asm("_binary_model_espdl_start");

dl::Model *model = new dl::Model(
    (const char *)model_espdl_start,
    fbs::MODEL_LOCATION_IN_FLASH_RODATA,
    0,                                   // max_internal_size — 0 = auto
    dl::MEMORY_MANAGER_GREEDY,
    nullptr,                             // encryption key
    true                                 // param_copy: copy weights to PSRAM
);

// Single-input run
model->run(input_tensor, RUNTIME_MODE_SINGLE_CORE);   // or DUAL_CORE
auto &outputs = model->get_outputs();   // std::map<std::string, TensorBase *>

// Multi-input run
std::map<std::string, dl::TensorBase *> inputs = {{"input", tensor}};
model->run(inputs);
```

**Memory model**: with `param_copy=true` (default), weights are copied
from flash rodata into PSRAM at boot — inference runs fully from PSRAM
(fast). With `param_copy=false`, weights stream from flash during
inference (saves PSRAM, costs ~2–3× latency). For a ≤200 KB model on
the XIAO Sense's 8 MB PSRAM, always use `param_copy=true`.

`internal_size=0` lets the planner decide how much internal SRAM (vs
PSRAM) to use for activations.

## Loading a model — SPIFFS partition path

For OTA-updatable models without re-flashing the full firmware. Reserve
a partition (e.g. our `model` slot in `melody-detector/partitions.csv`)
and load by label:

```cpp
dl::Model *model = new dl::Model("model", fbs::MODEL_LOCATION_IN_FLASH_PARTITION);
```

## Quantization with ESP-PPQ

### Install

```bash
# uv project (CPU-only training host)
uv pip install "esp-ppq[cpu]" --torch-backend=cpu

# CUDA training host
uv pip install esp-ppq
```

### Two entry points

```python
from esp_ppq import espdl_quantize_onnx, espdl_quantize_torch
```

Use `espdl_quantize_onnx` when you have an ONNX file (cleaner pipeline
boundary). Use `espdl_quantize_torch` to skip the ONNX hop.

### `espdl_quantize_onnx` recipe

```python
from esp_ppq import espdl_quantize_onnx
from torch.utils.data import DataLoader

calib_loader = DataLoader(
    calibration_dataset,    # ~512–1024 samples drawn from training distribution
    batch_size=16,
    shuffle=False,
)

espdl_quantize_onnx(
    onnx_import_file="model.onnx",
    espdl_export_file="model.espdl",
    calib_dataloader=calib_loader,
    calib_steps=32,                       # 32 batches × 16 = 512 samples
    input_shape=[1, 3, 32, 32],           # NCHW for our 32×32 ROI classifier
    target="esp32s3",                     # required — selects Espressif backend
    num_of_bits=8,                        # INT8; mix INT16 per layer if needed
    collate_fn=lambda x: x[0].to(device), # extract tensor from (img, label) tuple
    device="cpu",                         # or "cuda" on the 4090 host
)
```

Output is **three files** in the export directory:

- `model.espdl` — FlatBuffers binary for firmware
- `model.info` — human-readable layer dump with per-layer sensitivity
- `model.json` — quantization metadata

## Quantization gotchas (image classifiers)

### Per-tensor only on ESP32-S3

The ESP32-S3 ISA supports **per-tensor symmetric INT8 with
power-of-2 scales** for all layers. ESP32-P4 supports per-channel for
Conv/GEMM, ESP32-S3 does not. Practical impact:

- Avoid first-conv layers with high per-channel weight variance — the
  shared per-tensor scale flattens dynamic range
- For a classifier, this is usually fine; for a detector, expect more
  pain
- Check `.info` after quantization for per-layer sensitivity scores

### FC tail layer precision

If accuracy degrades at the final fully-connected layer (common with
heavy classification heads), pass `num_of_bits=16` for that layer via
ESP-PPQ's layer-wise config. The `.espdl` format supports mixed
INT8/INT16 per layer.

### BatchNorm folding

ESP-PPQ folds BatchNorm into the preceding Conv automatically before
quantization. You **don't** need to fold manually in PyTorch. If you
export with `torch.onnx.export(..., training=torch.onnx.TrainingMode.EVAL)`,
PyTorch will already fold BN into Conv in the exported graph — that's
fine too.

### Input normalization

ESP-DL's runtime does **not** apply normalization — it passes raw byte
values from the image buffer to `model->run()`. You must either:

- **Bake normalization into the ONNX graph** (recommended for v1) — the
  PyTorch model includes `Normalize`/`Sub`/`Div` ops, which fold into
  the first Conv weights during quantization. The runtime input tensor
  type becomes `int8_t` and ESP-DL handles scale/zero-point internally.
- **Apply in firmware before run()** — extra C++ glue, harder to keep
  in sync with the trained model.

### ROUND_HALF_UP rounding mode

ESP32-S3's INT8 rounding is `ROUND_HALF_UP`, which differs from
PyTorch's default banker's rounding. Expect **0.5–1% accuracy drop** at
the boundary even after calibration. Plan for this in acceptance
criteria.

## Latency expectations

No published benchmarks exist for a 100 KB INT8 classifier on
ESP32-S3 end-to-end. Kernel-level data points from
`operator_performance.md`:

| Operation | Time @ 240 MHz |
|---|---|
| Single 32×16×3×3 depthwise-separable conv on ~20×20 feature map | ~500 μs |
| 33-output 3×3 conv on 19×23 feature map | ~3.5 ms |

Community reports for MobileNetV2-family INT8 classifiers on ESP32-S3:
**50–200 ms** end-to-end. A bespoke small CNN (~100 KB, 2–3 conv layers,
32×32 input) should be **15–60 ms per ROI**.

> **Latency planning**: a 56-ROI sequential batch (32×32 each) at
> 15–60 ms per ROI is **840 ms – 3.4 s** total — likely above a 250 ms
> end-to-end budget. Mitigations:
>
> 1. **CV pre-filter**: only run inference on cells where the classical
>    CV stage detected ink above an empty-threshold. Most kid drawings
>    leave most cells empty; expect 10–20 active ROIs out of 56.
> 2. **`RUNTIME_MODE_DUAL_CORE`** in the `model->run()` call — splits
>    work across both LX7 cores.
> 3. **Aggressive model shrink**: 2 conv layers, narrow channels (8–16),
>    grouped convs.
>
> The PIE SIMD gives 2–10× over TFLM on the same model, so if a TFLM
> baseline is 500 ms, ESP-DL hits 50–100 ms. Per-ROI is fast; **the
> sequential batch is the binding constraint** — not the model.

## Known issues

| Issue | Impact | Mitigation |
|---|---|---|
| #295 (open) — IDF v6.0 compile failure | doesn't affect IDF v5.4 | stay on v5.4 until fixed |
| #187 (closed) — `espdl_quantize_onnx` TypeError on Gather/dynamic-index | hits custom YOLO variants, not standard classifiers | avoid `Gather` on constant indices; prefer `Slice` |
| Apple Silicon training host | no ESP-DL-specific issues | calibration on CPU is fast enough; use `--torch-backend=cpu` |
| Schema break at 3.1.0 | re-export needed when bumping ESP-DL | pin `>=3.3.2` and re-quantize on upgrade |

## Sources

- [github.com/espressif/esp-dl](https://github.com/espressif/esp-dl) — README, source, examples
- [components.espressif.com/components/espressif/esp-dl](https://components.espressif.com/components/espressif/esp-dl) — registry version
- [docs.espressif.com — ESP-DL: How to load/test/profile model](https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_load_test_profile_model.html)
- [docs.espressif.com — ESP-DL: How to quantize model](https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_quantize_model.html)
- [docs.espressif.com — ESP-DL: How to deploy MobileNetV2](https://docs.espressif.com/projects/esp-dl/en/latest/tutorials/how_to_deploy_mobilenetv2.html)
- [github.com/espressif/esp-ppq](https://github.com/espressif/esp-ppq)
- [esp-dl/operator_performance.md](https://github.com/espressif/esp-dl/blob/master/operator_performance.md)
