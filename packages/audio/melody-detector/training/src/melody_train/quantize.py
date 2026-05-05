"""Quantize the exported ONNX model to ``.espdl`` via ESP-PPQ.

Wraps ``espdl_quantize_onnx`` with the project-specific calibration
loader (drawn from the synthetic generator's training subset) and the
ESP32-S3 backend settings. Produces three files in the output directory:

- ``<out>.espdl`` — FlatBuffers binary embedded by the firmware via
  ``EMBED_FILES`` + ``target_add_aligned_binary_data``
- ``<out>.info``  — human-readable per-layer dump with sensitivity scores
- ``<out>.json``  — quantization metadata (scales, zero-points)

ESP32-S3 only supports per-tensor symmetric INT8 with power-of-2 scales;
the model's depthwise-separable design and small parameter count keep
per-tensor weight variance low enough to quantize cleanly.

Calibration draws from the **train** subset of the same dataset used
during training so the val subset stays unseen for downstream accuracy
checks. Default ``calib_steps=32`` × ``batch_size=16`` = 512 samples,
matching the recommendation in
``docs/reference/esp-dl-deployment.md``.

CLI entry-point at the bottom.
"""

from __future__ import annotations

import pathlib
import sys
from dataclasses import dataclass
from typing import Any

import click
import torch
from torch.utils.data import DataLoader

from melody_train.dataset import make_split
from melody_train.model import INPUT_CHANNELS, INPUT_SIZE


@dataclass(frozen=True)
class QuantizeConfig:
    onnx_path: pathlib.Path
    data_dir: pathlib.Path
    out_path: pathlib.Path
    calib_steps: int = 32
    batch_size: int = 16
    target: str = "esp32s3"
    num_of_bits: int = 8
    val_fraction: float = 0.1
    seed: int = 0
    device: str = "cpu"


@dataclass(frozen=True)
class QuantizeResult:
    espdl_path: pathlib.Path
    info_path: pathlib.Path
    json_path: pathlib.Path
    target: str
    num_of_bits: int


# ----------------------------------------------------- calibration loader ----


def _calibration_loader(cfg: QuantizeConfig) -> DataLoader:
    """Build a DataLoader over the train subset for ESP-PPQ calibration.

    Returns float32 tensors in ``[0, 255]`` (the ONNX graph's normalization
    Sub/Div ops handle the scale-down to ``[-1, 1]``). Iteration is
    deterministic — no shuffling — so calibration is reproducible.
    """
    _, train_subset, _ = make_split(
        cfg.data_dir,
        val_fraction=cfg.val_fraction,
        seed=cfg.seed,
    )
    return DataLoader(
        train_subset,
        batch_size=cfg.batch_size,
        shuffle=False,
        num_workers=0,
        drop_last=False,
    )


def _calib_collate(batch: tuple[torch.Tensor, torch.Tensor], device: str) -> torch.Tensor:
    """Strip labels; cast uint8 → float32; move to device.

    ESP-PPQ invokes ``collate_fn`` after the DataLoader has already
    stacked the batch, so ``batch`` here is the ``(images, labels)``
    tuple — not the per-sample list. Matches the ONNX model's expected
    input: float32 in ``[0, 255]``.
    """
    images = batch[0]
    return images.float().to(device)


# --------------------------------------------------------------- quantize ----


def quantize(cfg: QuantizeConfig) -> QuantizeResult:
    """Quantize ``cfg.onnx_path`` to ``cfg.out_path`` using ESP-PPQ."""
    if not cfg.onnx_path.exists():
        raise FileNotFoundError(f"ONNX file not found: {cfg.onnx_path}")

    try:
        from esp_ppq.api import espdl_quantize_onnx
    except ImportError as exc:
        raise ImportError(
            "esp-ppq is not installed. Run `uv sync --extra quantize` "
            "(or `just sync-quantize`) to install ESP-PPQ + ONNX deps."
        ) from exc

    cfg.out_path.parent.mkdir(parents=True, exist_ok=True)

    loader = _calibration_loader(cfg)
    device = cfg.device

    espdl_quantize_onnx(
        onnx_import_file=str(cfg.onnx_path),
        espdl_export_file=str(cfg.out_path),
        calib_dataloader=loader,
        calib_steps=cfg.calib_steps,
        input_shape=[1, INPUT_CHANNELS, INPUT_SIZE, INPUT_SIZE],
        target=cfg.target,
        num_of_bits=cfg.num_of_bits,
        collate_fn=lambda batch: _calib_collate(batch, device),
        device=device,
    )

    info_path = cfg.out_path.with_suffix(".info")
    json_path = cfg.out_path.with_suffix(".json")

    return QuantizeResult(
        espdl_path=cfg.out_path,
        info_path=info_path,
        json_path=json_path,
        target=cfg.target,
        num_of_bits=cfg.num_of_bits,
    )


# --------------------------------------------------------- CLI entry-point ---


@click.command()
@click.option(
    "--onnx",
    "onnx_path",
    type=click.Path(dir_okay=False, path_type=pathlib.Path),
    default=pathlib.Path("runs/model.onnx"),
    show_default=True,
    help="Source ONNX file (produced by melody_train.export).",
)
@click.option(
    "--data",
    "data_dir",
    type=click.Path(exists=True, file_okay=False, path_type=pathlib.Path),
    required=True,
    help="Dataset directory used for calibration (train subset only).",
)
@click.option(
    "--out",
    "out_path",
    type=click.Path(dir_okay=False, path_type=pathlib.Path),
    default=pathlib.Path("runs/model.espdl"),
    show_default=True,
    help="Destination .espdl file (also produces .info and .json siblings).",
)
@click.option("--calib-steps", type=int, default=32, show_default=True)
@click.option("--batch-size", type=int, default=16, show_default=True)
@click.option("--target", type=str, default="esp32s3", show_default=True)
@click.option("--num-of-bits", type=int, default=8, show_default=True)
@click.option("--val-fraction", type=float, default=0.1, show_default=True)
@click.option("--seed", type=int, default=0, show_default=True)
@click.option("--device", type=str, default="cpu", show_default=True)
def _cli(**kwargs: Any) -> None:
    cfg = QuantizeConfig(**kwargs)
    result = quantize(cfg)
    click.echo(
        f"espdl -> {result.espdl_path} (target={result.target}, num_of_bits={result.num_of_bits})"
    )
    click.echo(f"info  -> {result.info_path}")
    click.echo(f"json  -> {result.json_path}")


def main(argv: list[str] | None = None) -> int:
    """CLI entry point. Returns process exit code."""
    try:
        _cli.main(args=argv, standalone_mode=False)
    except click.exceptions.UsageError as exc:
        click.echo(f"Error: {exc.format_message()}", err=True)
        return 2
    except click.exceptions.Abort:
        return 130
    except FileNotFoundError as exc:
        click.echo(f"Error: {exc}", err=True)
        return 1
    except ImportError as exc:
        click.echo(f"Error: {exc}", err=True)
        return 1
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
