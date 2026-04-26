# Cloud Vision APIs as Labeling Oracles

Reference for using cloud vision-language models to label images for
supervised training. Useful when you need a fast validation set or want to
bootstrap labels for fine-tuning a small on-device model.

Last reviewed: 2026-04-26.

## Decision shortcut

| Need | Recommended service |
|---|---|
| Spatial labels (bounding boxes, points, segments) | **Gemini Robotics-ER 1.6** |
| General descriptive labels | Gemini 3.x Flash (free tier) or Claude Sonnet 4.6 |
| OCR / structured text from images | Gemini 3.x Pro |
| Self-hosted / private data only | Florence-2, Qwen2-VL, or YOLO-World (open weights) |

## Gemini Robotics-ER 1.6

Purpose-built for spatial reasoning and embodied tasks. Returns normalized
bounding boxes and segmentation masks in a stable JSON format — ideal as a
labeling oracle for object-detection training data.

**Output format**

```json
[
  {"box_2d": [ymin, xmin, ymax, xmax], "label": "notehead_line_1"},
  {"box_2d": [...], "label": "notehead_space_2"}
]
```

Coordinates are normalized to 0–1000 (independent of image resolution).
Output is capped at 25 objects per image.

**Strengths**
- Stable structured output — no prompt-engineering struct hacks needed.
- Spatial reasoning quality is genuinely better than general-purpose Gemini
  for "where is each X" tasks.
- Optional segmentation masks alongside boxes.
- Single API call gives detection + classification.

**Weaknesses**
- Hard cap of 25 objects per image — not suitable for dense scenes.
- 0–1000 normalization quirk to handle in code.
- Pro tier required for reliable output on hard cases (paid as of April
  2026).

**Pricing (April 2026)**
- Available via Gemini API: `generativelanguage.googleapis.com/v1beta/...`
- Auth: `x-goog-api-key` header.
- Pro pricing: see Gemini 3.x Pro tier.

**When to pick**
- Bootstrapping labels for a new computer-vision dataset.
- Building a validation set quickly (50–500 images).
- Spatial tasks where coordinates matter, not just class labels.

**Docs**
- [Gemini Robotics overview](https://ai.google.dev/gemini-api/docs/robotics-overview)
- [Vertex AI bounding box detection](https://docs.cloud.google.com/vertex-ai/generative-ai/docs/bounding-box-detection)

## Gemini 3.x Flash / Pro (general vision)

The mainline Gemini multimodal models, useful for descriptive captions, OCR,
and free-form visual Q&A.

**Pricing (April 2026)**
- Gemini 3.1 Pro: $2.00 / $12.00 per 1M tokens (≤200K context); $4.00 /
  $18.00 above.
- Gemini 3.1 Flash-Lite: $0.25 / $1.50 per 1M tokens.
- Image input: 560 tokens per image regardless of model.
- Free tier: Flash models still free with reduced daily quotas. **Pro models
  removed from free tier on April 1, 2026.**

**When to pick**
- Generic image understanding ("describe this", "is this a cat?").
- OCR — Pro is the strongest mainstream OCR model in 2026.
- Cheap, high-volume tagging — Flash-Lite at $0.25/M input is the cheapest
  Tier-1 vision model.

**Docs**
- [Gemini API pricing](https://ai.google.dev/gemini-api/docs/pricing)
- [Gemini API models](https://ai.google.dev/gemini-api/docs/models)

## Claude Vision (Sonnet 4.6 / Opus 4.7)

Anthropic's multimodal models. Excellent at reading messy human handwriting,
diagrams, and reasoning about images.

**Strengths**
- Strong at handwriting, diagrams, charts.
- Long context window (Opus 4.7: 200K, Sonnet 4.6: similar).
- Reliable structured output via tool use / JSON mode.

**Weaknesses**
- No native bounding-box output — coordinates are best-effort via prompting.
- More expensive than Gemini Flash for bulk labeling.

**When to pick**
- Tasks involving handwriting, sketches, or messy real-world documents.
- When Anthropic SDK is already in the project.

## OpenAI GPT-4o / GPT-5.x Vision

Capable, broadly comparable to Gemini Pro on most benchmarks. No native
spatial-output mode equivalent to Gemini Robotics-ER.

**When to pick**
- Existing OpenAI integration.
- Otherwise no specific advantage for spatial labeling.

## Open-source / self-hosted alternatives

For projects where data privacy or offline operation matters.

| Model | Strength | Notes |
|---|---|---|
| **Florence-2** (Microsoft) | Bounding boxes + segmentation, small footprint | 230M / 770M params, runs on a 4090 easily |
| **Qwen2-VL** (Alibaba) | Strong general VLM, supports tool use | 2B / 7B / 72B params |
| **YOLO-World** (Tencent) | Open-vocabulary detection | Real-time on GPU; no LLM in the loop |
| **OWLv2** (Google) | Open-vocabulary detection, text queries | Older but solid |
| **Grounding DINO** | Text-prompted detection | Strong for "find all X" tasks |

**When to pick**
- Sensitive data that can't leave your infrastructure.
- Bulk labeling where API costs would be prohibitive.
- Reproducibility — the model and weights are pinned.

## Cost worked example: 1000-image validation set

Labeling 1000 images for "find every notehead, return position":

| Service | Cost | Notes |
|---|---|---|
| Gemini Robotics-ER 1.6 (Pro) | ~$10 | One call/image, ~3K tokens per response |
| Gemini 3.1 Flash-Lite | ~$1 | Free tier covers it if rate limits allow |
| Claude Sonnet 4.6 | ~$15 | Higher quality on messy drawings |
| Florence-2 (self-hosted) | $0 (compute only) | RTX 4090 batches 16+ images at once |

For a hobby project, free-tier Gemini Flash plus a few hundred Pro calls on
hard cases is the practical sweet spot.

## Workflow patterns

### Bootstrap labels → fine-tune small model

1. Hand-label 50 seed images.
2. Use Robotics-ER to label 1000–10000 more (oracle labels).
3. Spot-check 10% by hand; reject obvious errors.
4. Train a small on-device model on the bulk dataset.
5. Use the seed set + a held-out set as the true validation.

### Oracle-as-baseline

Quote on-device model accuracy as a fraction of oracle accuracy. "Our
200KB on-device model hits 94% of Gemini Robotics-ER's accuracy at 0%
of the per-inference cost" is a more compelling number than raw mAP.

### Active learning

Use the oracle to label only images where the on-device model is uncertain.
Reduces oracle API spend by an order of magnitude.
