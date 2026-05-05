"""Export the trained ROI classifier from PyTorch to ONNX.

Wraps :class:`MelodyClassifier` in a small shim that takes float32
input in ``[0, 255]`` and applies the same ``(x - 128) / 128``
normalization as ``MelodyClassifier._normalize`` — but as explicit
``Sub``/``Div`` ops on float32 input so they survive ONNX export. ESP-PPQ
folds those ops into the first Conv weights during INT8 quantization, so
the on-device runtime can pass raw camera bytes (cast to float on the
host side of ESP-DL) straight into ``model->run()`` without any C++
pre-processing.

ONNX export uses ``opset_version=17`` (ESP-PPQ requirement) and
``training=torch.onnx.TrainingMode.EVAL`` so BatchNorm folds into the
preceding Conv in the exported graph. ``dynamic_axes`` flexes the batch
dimension so ESP-PPQ can run calibration with a different batch size.

CLI entry-point at the bottom.
"""

from __future__ import annotations

import pathlib
import sys
from dataclasses import dataclass
from typing import Any

import click
import torch
from torch import nn

from melody_train.model import INPUT_CHANNELS, INPUT_SIZE, MelodyClassifier


@dataclass(frozen=True)
class ExportConfig:
    checkpoint_path: pathlib.Path
    out_path: pathlib.Path
    opset_version: int = 17
    input_name: str = "input"
    output_name: str = "output"


@dataclass(frozen=True)
class ExportResult:
    onnx_path: pathlib.Path
    opset_version: int
    input_name: str
    output_name: str


# ----------------------------------------------------------------- shim ----


class _ExportShim(nn.Module):
    """Wrap a trained MelodyClassifier for ONNX export with in-graph normalization.

    Accepts float32 input in ``[0, 255]`` (the natural representation for
    raw camera bytes) and applies ``(x - 128) / 128`` as explicit Sub/Div
    ops before the model body. These ops fold into the first Conv weights
    during ESP-PPQ quantization.

    The shim bypasses :meth:`MelodyClassifier._normalize` because that
    branch only triggers for uint8 input, and ONNX export with uint8
    inputs emits Cast ops that ESP-PPQ does not handle uniformly across
    versions.
    """

    def __init__(self, model: MelodyClassifier) -> None:
        super().__init__()
        self.model = model

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = (x - 128.0) / 128.0
        x = self.model.block1(x)
        x = self.model.block2(x)
        x = self.model.block3(x)
        x = self.model.pool(x)
        x = torch.flatten(x, 1)
        x = self.model.classifier(x)
        return x


# --------------------------------------------------------------- export ----


def _load_checkpoint(checkpoint_path: pathlib.Path) -> MelodyClassifier:
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
    payload = torch.load(checkpoint_path, weights_only=True, map_location="cpu")
    if "model_state" not in payload:
        raise ValueError(
            f"Checkpoint at {checkpoint_path} missing 'model_state' key — "
            "expected the dict format saved by melody_train.train"
        )
    model = MelodyClassifier()
    model.load_state_dict(payload["model_state"])
    model.eval()
    return model


def export(cfg: ExportConfig) -> ExportResult:
    """Export the checkpoint at ``cfg.checkpoint_path`` to ONNX."""
    model = _load_checkpoint(cfg.checkpoint_path)
    shim = _ExportShim(model).eval()

    cfg.out_path.parent.mkdir(parents=True, exist_ok=True)

    dummy = torch.randint(
        0,
        256,
        (1, INPUT_CHANNELS, INPUT_SIZE, INPUT_SIZE),
        dtype=torch.float32,
    )

    with torch.no_grad():
        # dynamo=False selects the legacy TorchScript-based exporter, which
        # natively emits opset 17 — the new dynamo exporter targets opset 18
        # and fails when asked to downgrade to 17 because of a Reshape node
        # without a constant `axes` initializer. ESP-PPQ pins opset 17, so
        # the legacy path is the right one here.
        torch.onnx.export(
            shim,
            (dummy,),
            str(cfg.out_path),
            opset_version=cfg.opset_version,
            training=torch.onnx.TrainingMode.EVAL,
            input_names=[cfg.input_name],
            output_names=[cfg.output_name],
            dynamic_axes={
                cfg.input_name: {0: "batch"},
                cfg.output_name: {0: "batch"},
            },
            do_constant_folding=True,
            dynamo=False,
        )

    return ExportResult(
        onnx_path=cfg.out_path,
        opset_version=cfg.opset_version,
        input_name=cfg.input_name,
        output_name=cfg.output_name,
    )


# --------------------------------------------------------- CLI entry-point ---


@click.command()
@click.option(
    "--checkpoint",
    "checkpoint_path",
    type=click.Path(dir_okay=False, path_type=pathlib.Path),
    default=pathlib.Path("runs/best.pt"),
    show_default=True,
    help="Trained checkpoint produced by melody_train.train.",
)
@click.option(
    "--out",
    "out_path",
    type=click.Path(dir_okay=False, path_type=pathlib.Path),
    default=pathlib.Path("runs/model.onnx"),
    show_default=True,
    help="Destination ONNX file.",
)
@click.option(
    "--opset",
    "opset_version",
    type=int,
    default=17,
    show_default=True,
    help="ONNX opset version (ESP-PPQ requires 17).",
)
def _cli(**kwargs: Any) -> None:
    cfg = ExportConfig(**kwargs)
    result = export(cfg)
    click.echo(
        f"onnx -> {result.onnx_path} "
        f"(opset={result.opset_version}, "
        f"input={result.input_name}, output={result.output_name})"
    )


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
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
