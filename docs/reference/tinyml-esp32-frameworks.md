# TinyML Inference Frameworks for ESP32-Class Targets

Reference summary of on-device ML inference frameworks evaluated for the
melody-detector project (XIAO ESP32-S3 Sense). Useful as a starting point for
future ESP32 / ESP32-S3 projects that need on-device vision or audio
inference.

Last reviewed: 2026-04-26.

## At a glance

| Framework | Vendor | License | ESP32-S3 status | Best inference perf on S3 | Training flow |
|---|---|---|---|---|---|
| **ESP-DL v3.x** | Espressif | Apache 2.0 | First-class, production | Yes (PIE assembly kernels) | PyTorch → ONNX → ESP-PPQ → `.espdl` |
| **TFLite Micro + ESP-NN** | Google + Espressif | Apache 2.0 | Mature | Good | Keras/TF → TFLite converter → `.tflite` |
| **LiteRT (TFLite successor)** | Google | Apache 2.0 | Targets phone/Pi/desktop, not MCU | N/A on MCU | TF/Keras/PyTorch → LiteRT converter |
| **ExecuTorch** | Meta (PyTorch) | BSD-3 | Early ESP32 backend (2024+) | Behind ESP-DL on S3 | PyTorch → `torch.export` → `.pte` |
| **Edge Impulse** | Edge Impulse / Qualcomm | Proprietary SaaS (free tier) | Officially supported | Wraps TFLM (slower than ESP-DL) | Web UI → exports as Arduino lib / IDF component |

## ESP-DL

Espressif's official deep learning library, designed for ESP32-S3 and later
SoCs that have PIE (Packed-SIMD-like) vector instructions.

**Strengths**
- Hand-tuned assembly kernels for conv / matmul on ESP32-S3 deliver 2–10×
  speedup over generic TFLM kernels.
- ESP-PPQ (forked from PPQ, the PyTorch quantization toolkit) provides
  post-training INT8/INT16 quantization with calibration, mixed precision,
  and per-layer sensitivity analysis.
- Espressif ships a model zoo with pretrained detectors (face, person,
  pedestrian) suitable for fine-tuning.
- Tightly integrated with ESP-IDF — ships as a managed component.

**Weaknesses**
- Custom `.espdl` model format is ESP32-only; not portable to phones / Pi.
- Documentation is improving but you will read source occasionally.
- Smaller community than TFLM / LiteRT.

**When to pick**
- Production ESP32-S3 deployments where inference latency matters.
- Projects already inside the ESP-IDF ecosystem.
- Showcases that benefit from "runs natively on the SoC's vector unit"
  framing.

**Repo / docs**
- [github.com/espressif/esp-dl](https://github.com/espressif/esp-dl)
- [github.com/espressif/esp-ppq](https://github.com/espressif/esp-ppq)

## TFLite Micro + ESP-NN

The classic TinyML stack: TensorFlow Lite for Microcontrollers with
Espressif's ESP-NN providing optimized kernels for ESP32 / ESP32-S3.

**Strengths**
- Largest community, most tutorials, most reference models.
- Mature toolchain: `.tflite` flatbuffer model is a stable target.
- Works on classic ESP32 (no S3 vector unit required) — only option for
  older boards.
- Edge Impulse exports use this stack under the hood.

**Weaknesses**
- Slower than ESP-DL on ESP32-S3 because kernel optimization stops at
  ESP-NN's level (no PIE assembly).
- Google has shifted active investment to LiteRT, which targets larger
  devices; TFLM is in maintenance mode.

**When to pick**
- Projects on classic ESP32 (no S3) that need ML inference.
- Projects where the existing TFLM ecosystem (model zoo, tutorials)
  outweighs raw speed.

**Repo**
- [github.com/tensorflow/tflite-micro](https://github.com/tensorflow/tflite-micro)
- [github.com/espressif/esp-nn](https://github.com/espressif/esp-nn)

## LiteRT (TensorFlow Lite successor)

Google's renamed and modernized edge runtime. Includes the new
`CompiledModel` API for hardware acceleration, supports loading PyTorch
models alongside TF, and powers on-device LLMs (LiteRT-LM) on phones and
Chromebooks.

**Strengths**
- Modern API, active development, multi-framework input.
- Targets phones, Pi-class Linux, web (WASM), desktop.
- LiteRT-LM enables on-device GenAI on capable edge devices.

**Weaknesses**
- **Does not target microcontroller-class hardware.** No ESP32 backend.
- Confusion risk: "LiteRT" sometimes used loosely to mean "TFLite for
  edge" — for MCUs, TFLite Micro is still the relevant stack.

**When to pick**
- Edge ML on phones, Pi, embedded Linux, web.
- Not for ESP32 / ESP32-S3 firmware.

**Repo**
- [github.com/google-ai-edge/LiteRT](https://github.com/google-ai-edge/LiteRT)

## ExecuTorch

PyTorch's official edge runtime, replacing TorchScript / PyTorch Mobile. Uses
the new PT2 export flow (`torch.export`) to produce `.pte` model files that
run on a small C++ runtime.

**Strengths**
- PyTorch end-to-end — no ONNX detour.
- Same `.pte` model can run on phone, Pi, ESP32 — strong portability story.
- Modern quantization via `torchao` and PT2E.
- Active development, large PyTorch community.

**Weaknesses**
- ESP32 backend is recent (2024+) and kernel coverage lags ESP-DL.
- Inference speed on ESP32-S3 is behind ESP-DL because PIE-vectorized
  kernels are not yet at parity.
- Expect to occasionally hit "operator not implemented" and need
  workarounds.

**When to pick**
- Cross-target deployments (same model on phone + ESP32).
- Pure-PyTorch pipelines where avoiding ONNX is a hard requirement.
- Projects where being on the bleeding edge is acceptable.

**Repo**
- [github.com/pytorch/executorch](https://github.com/pytorch/executorch)

## Edge Impulse

Web-based platform for collecting data, training, and deploying TinyML
models. Generates Arduino libraries and ESP-IDF components.

**Strengths**
- Lowest friction end-to-end workflow: web UI for data labeling, training,
  quantization, deployment.
- Officially supports XIAO ESP32-S3 Sense.
- Free Developer (Community) tier covers personal / hobby / educational
  projects with full feature access.
- Good fit for non-ML-experts on a team.

**Weaknesses**
- Proprietary SaaS — free-tier projects are public (anyone with the URL can
  view the dataset and model).
- Acquired by Qualcomm in 2025; long-term roadmap and free-tier terms may
  shift.
- Generated firmware uses TFLM under the hood, so inference on ESP32-S3 is
  slower than ESP-DL.
- Lock-in: dataset and trained model live on their platform; export is
  possible but not seamless.

**When to pick**
- Rapid prototyping, hackathons, classroom projects.
- Teams without an ML engineer.
- Projects where you accept SaaS lock-in for workflow speed.

**Site**
- [edgeimpulse.com](https://edgeimpulse.com)

## Decision matrix for new projects

```
Need to run on classic ESP32 (no S3)?     → TFLite Micro + ESP-NN
Need cross-platform model (phone + MCU)?  → ExecuTorch
Need fastest inference on ESP32-S3?       → ESP-DL
Need lowest workflow friction?            → Edge Impulse (free tier)
Targeting phone / Pi / desktop edge?      → LiteRT
```

## Useful adjacent tools

- **PPQ / ESP-PPQ** — quantization toolkit (PyTorch-based), backend for
  ESP-DL.
- **ONNX Runtime Web / Mobile** — alternative inference runtime, does not
  target MCUs but useful for shared models on companion apps.
- **MLflow / Weights & Biases** — experiment tracking; orthogonal to the
  framework choice.
- **Roboflow** — dataset management and augmentation; can export to TFLite,
  ONNX, ExecuTorch.
