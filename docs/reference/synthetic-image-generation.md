# Synthetic Image Generation for ML Training

Reference for generating labeled training data when real data is scarce or
expensive to label. Aimed at small computer-vision tasks where the
input domain is structured enough to render programmatically.

Last reviewed: 2026-04-26.

## Decision tree

```
Is the input domain structured (geometry, layout, known classes)?
├── Yes  → Programmatic rendering + augmentation (cheapest, free labels)
└── No   → Diffusion-based generation (expensive, needs labeling step)

Need stylistic diversity beyond augmentations?
└── Add diffusion-generated images as ~5–10% of dataset.

Need real-world appearance fidelity?
└── Validate with a small real-world holdout set; bridge with domain
    randomization.
```

## Programmatic rendering

Generate images deterministically from code. Labels come for free because
the rendering parameters are the labels.

**Tools**
- **PIL / Pillow** — simple 2D rasterization, fast.
- **Cairo** — vector graphics, anti-aliased, good for clean diagrams.
- **SVG → PNG (cairosvg, resvg)** — author in vector, render at multiple
  scales.
- **Blender (bpy)** — 3D scenes, physically-based lighting; overkill for 2D
  but unbeatable for 3D.
- **NVISII / OmniSim / Isaac Sim** — physics-based synthesis for robotics.

**Strengths**
- Free at scale — generate millions of images.
- Perfect labels — no annotation cost.
- Reproducible — the dataset is `git clone && python gen.py`.
- Full control over class balance, edge cases, hard examples.

**Weaknesses**
- Domain gap: synthetic images may not match real-world appearance.
- Authoring the renderer takes engineering time upfront.
- Diversity is bounded by what you code.

**When to pick**
- Structured domains: charts, diagrams, board games, sheet music, UIs,
  barcodes, AR markers.
- Geometric scenes with known objects.
- Anything where you can describe a sample as code.

## Augmentation libraries

Multiplicative on top of programmatic rendering. Apply photometric and
geometric transforms to expand a small base set.

| Library | Strengths | Notes |
|---|---|---|
| **albumentations** | Fastest, broadest set of transforms, handles bboxes / masks / keypoints | de-facto standard in 2026 |
| **kornia** | GPU-accelerated, differentiable, integrates with PyTorch | Use when augmentations need to run inside the training loop |
| **imgaug** | Older, slower than albumentations | Legacy projects |
| **torchvision.transforms.v2** | Stdlib, GPU-friendly | Sufficient for simple cases |

**Standard transform set for paper/document tasks**
- Random affine (rotation ±5°, scale 0.9–1.1, shear)
- Perspective warp (camera not perpendicular)
- Brightness / contrast / gamma
- Gaussian / motion blur (camera focus, motion)
- Gaussian noise (sensor noise)
- JPEG compression (camera artifacts)
- Random erasing / coarse dropout (occlusion)
- Color jitter / hue shift (lighting variation)
- Paper texture overlay (synthetic + real paper backgrounds)

## Diffusion-based generation

Use generative models to produce images that programmatic rendering can't
synthesize naturally — stylistic variation, photorealistic textures, novel
poses.

| Model | Pricing | Self-hostable? | Notes |
|---|---|---|---|
| **Imagen 4 Fast** | $0.02 / image | No (Gemini API) | 1024×1024 default, up to 2K. Cheapest Tier-1 |
| **Imagen 4 Standard** | $0.04 / image | No | Higher quality |
| **Imagen 4 Ultra** | $0.06 / image | No | Best quality, photorealism |
| **FLUX.1 dev** | Free (open weights) | Yes (RTX 4090: 5–10 s/image) | State-of-the-art open model in 2026 |
| **SDXL / SD 3.x** | Free (open weights) | Yes | Older but well-supported, lots of LoRAs |
| **GPT Image 1.5 (OpenAI)** | ~$0.04–0.17 / image (resolution-tier) | No | Strong instruction-following |
| **Grok Imagine** | API, varies | No | Growing capability |

**Strengths**
- Photorealistic output, stylistic diversity.
- Can produce things programmatic rendering can't (lighting, materials,
  unusual angles).

**Weaknesses**
- **Labels are not free** — must label generated images post-hoc, or
  condition tightly on a known prompt.
- Cost adds up: 10K images at $0.02 each = $200.
- Can hallucinate domain-specific details (text, fingers, exact geometry).
- Reproducibility requires pinning model version + seed.

**When to pick**
- Domain has critical visual variety that augmentations can't capture.
- Adding ~5–10% of dataset for diversity, not the bulk.
- One-time generation of hero examples or hard-case demos.

### Conditional generation: ControlNet, IP-Adapter, image-to-image

Condition diffusion on a programmatically rendered template — get the
benefits of free labels and stylistic diversity together.

```
Programmatic render (clean template + ground truth labels)
         ↓
ControlNet conditioning (Canny / depth / pose)
         ↓
Diffusion model (FLUX, SD 3.x)
         ↓
Stylized image (still labeled by the template)
```

**When to pick**
- Best-of-both-worlds when you need labels AND stylistic variation.
- Requires self-hosted setup (ControlNet workflows are mature in
  ComfyUI / Forge but not generally available via cloud APIs).

## Domain randomization

Aggressively vary non-essential aspects of the synthetic scene during
generation: lighting, background textures, camera pose, color schemes,
distractors. Forces the model to learn invariant features.

**Standard knobs**
- Background: random natural images (ImageNet, Places365) behind the
  subject.
- Lighting: directional + ambient, random color temperature.
- Camera: perspective, distance, angle within plausible range.
- Distractors: random objects / shapes pasted into the scene.
- Color: full hue rotation if class-irrelevant.

**When to pick**
- Always, when shipping a synthetic-only model to the real world.
- Especially important for 3D / physics-based synthesis.

## GAN-based synthesis

Largely superseded by diffusion in 2026. Mention only for completeness:
StyleGAN / pix2pix / CycleGAN. Still useful for narrow domains where you
have a small real dataset and want to expand it via image-to-image
translation.

## Cost-quality matrix

| Approach | Cost per 10K images | Label fidelity | Diversity | Recommended share |
|---|---|---|---|---|
| Programmatic + albumentations | ~$0 | Perfect | Bounded by code | 80–95% |
| Diffusion (Imagen 4 Fast) | $200 | Manual or oracle-labeled | High | 0–10% |
| Diffusion (FLUX self-hosted) | ~$0 (compute) | Manual or oracle-labeled | High | 0–15% |
| ControlNet on programmatic | ~$0 (compute) | Inherited from template | High | 0–15% |
| Real photographs + Robotics-ER labels | $10–50 (API) + collection time | Oracle-labeled | Real | 5–20% (validation set) |

A typical mix for a small-budget project: **90% programmatic + augmentation,
5% ControlNet-stylized, 5% real photos as validation.**

## Reproducibility checklist

- Pin random seeds in the generation script.
- Pin diffusion model versions and revisions (Hugging Face commit hash).
- Commit the generation script alongside the firmware repo, not the
  dataset itself.
- Document augmentation parameters (probability, range) in the dataset
  README.
- Hash the generated dataset (`sha256sum`) and store the digest with the
  trained model checkpoint.
