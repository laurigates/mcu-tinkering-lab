"""ROI classifier — small depthwise-separable CNN.

Designed to deploy via ESP-PPQ → ESP-DL on ESP32-S3 (per-tensor symmetric
INT8). Architecture choices (per ESP-DL deployment reference):

- In-graph input normalization: ``(x - 128) / 128`` so the
  normalization folds into the first conv during ESP-PPQ quantization
  and the on-device runtime can pass raw ``int8_t`` to ``model->run()``
  without any C++ pre-processing.
- Three blocks of (conv, BN, ReLU, MaxPool) with depthwise-separable
  convs in blocks 2 and 3 to keep parameter count and per-tensor
  weight variance low — the latter matters because ESP32-S3 only
  supports per-tensor symmetric INT8.
- Global average pool → small FC head to keep the tail compact.

The model is intentionally tiny (~5 KB INT8). The 32×32 ROI task is
mostly texture / shape discrimination on five classes, which doesn't
need depth. Latency budget on-device: 5–10 ms per ROI.
"""

from __future__ import annotations

import torch
from torch import nn

from melody_train.gen import CLASSES

NUM_CLASSES: int = len(CLASSES)
INPUT_SIZE: int = 32
INPUT_CHANNELS: int = 1


# ----------------------------------------------------------------- blocks ----


class _DepthwiseSeparable(nn.Module):
    """Depthwise (3x3, groups=in) + pointwise (1x1) conv block.

    Two BNs and one ReLU between the two convs match the MobileNet
    convention and quantize cleanly.
    """

    def __init__(self, in_ch: int, out_ch: int) -> None:
        super().__init__()
        self.depthwise = nn.Conv2d(in_ch, in_ch, kernel_size=3, padding=1, groups=in_ch, bias=False)
        self.bn1 = nn.BatchNorm2d(in_ch)
        self.relu1 = nn.ReLU(inplace=True)
        self.pointwise = nn.Conv2d(in_ch, out_ch, kernel_size=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_ch)
        self.relu2 = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.depthwise(x)
        x = self.bn1(x)
        x = self.relu1(x)
        x = self.pointwise(x)
        x = self.bn2(x)
        x = self.relu2(x)
        return x


# -------------------------------------------------------------- classifier ----


class MelodyClassifier(nn.Module):
    """Small CNN classifying 32×32 grayscale ROIs into the 5 melody classes.

    Forward accepts either uint8 input (the natural format from the
    synthetic generator and the on-device camera buffer) or pre-normalised
    float32 in [-1, 1]. Internal (uint8 → float [-1, 1]) cast happens
    inside the graph so the normalization survives ONNX export and folds
    into the first conv during INT8 quantization.
    """

    def __init__(self, num_classes: int = NUM_CLASSES) -> None:
        super().__init__()

        self.block1 = nn.Sequential(
            nn.Conv2d(INPUT_CHANNELS, 16, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2),  # 32 -> 16
        )
        self.block2 = nn.Sequential(
            _DepthwiseSeparable(16, 32),
            nn.MaxPool2d(kernel_size=2),  # 16 -> 8
        )
        self.block3 = nn.Sequential(
            _DepthwiseSeparable(32, 32),
            nn.MaxPool2d(kernel_size=2),  # 8 -> 4
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.classifier = nn.Linear(32, num_classes)

    @staticmethod
    def _normalize(x: torch.Tensor) -> torch.Tensor:
        """uint8 → float32 in [-1, 1] via ``(x - 128) / 128``.

        Float32 input is assumed already pre-normalised (caller's
        responsibility) and passed through unchanged. Kept as a static
        method so it stays a single graph op for ONNX export — ESP-PPQ
        folds this into the first conv weights cleanly during INT8
        quantization.
        """
        if x.dtype == torch.uint8:
            return (x.to(torch.float32) - 128.0) / 128.0
        return x

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self._normalize(x)
        x = self.block1(x)
        x = self.block2(x)
        x = self.block3(x)
        x = self.pool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x


# --------------------------------------------------------------- inspection ---


def parameter_count(model: nn.Module) -> int:
    """Total trainable parameter count."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


def estimated_int8_size_bytes(model: nn.Module) -> int:
    """Approximate INT8 weight footprint for budget gating.

    1 byte per parameter (post-quantization) plus an allowance for
    per-tensor scale + zero-point metadata (~16 bytes per parameter
    tensor) and BatchNorm parameters that fold into the preceding conv
    but still consume a bit of overhead during the export flow.
    """
    n_params = parameter_count(model)
    n_tensors = sum(1 for _ in model.parameters())
    return n_params + n_tensors * 16
