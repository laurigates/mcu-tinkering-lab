"""Training loop for the melody-detector ROI classifier.

Deterministic given ``cfg.seed``. Saves the best validation checkpoint
to ``cfg.out_dir / 'best.pt'`` and per-epoch history to
``cfg.out_dir / 'history.json'``. CLI entry-point at the bottom.
"""

from __future__ import annotations

import json
import pathlib
import random
import sys
from dataclasses import asdict, dataclass, field
from typing import Any

import click
import numpy as np
import torch
from torch import nn

from melody_train.dataset import make_dataloaders, make_split
from melody_train.model import MelodyClassifier


@dataclass(frozen=True)
class TrainConfig:
    data_dir: pathlib.Path
    out_dir: pathlib.Path
    epochs: int = 10
    batch_size: int = 64
    learning_rate: float = 1e-3
    weight_decay: float = 1e-4
    val_fraction: float = 0.1
    seed: int = 0
    num_workers: int = 0
    device: str = "cpu"


@dataclass(frozen=True)
class TrainResult:
    best_val_accuracy: float
    final_val_accuracy: float
    final_train_loss: float
    epochs_run: int
    checkpoint_path: pathlib.Path
    history: list[dict[str, float]] = field(default_factory=list)


def _seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def _evaluate(
    model: nn.Module,
    loader: torch.utils.data.DataLoader,
    device: torch.device,
    loss_fn: nn.Module,
) -> tuple[float, float]:
    """Returns (mean_loss, accuracy) over the entire loader."""
    model.eval()
    total_loss = 0.0
    total_correct = 0
    total_samples = 0
    with torch.no_grad():
        for images, labels in loader:
            images = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            logits = model(images)
            loss = loss_fn(logits, labels)
            total_loss += float(loss.item()) * images.shape[0]
            total_correct += int((logits.argmax(dim=1) == labels).sum().item())
            total_samples += images.shape[0]
    if total_samples == 0:
        return 0.0, 0.0
    return total_loss / total_samples, total_correct / total_samples


def train(cfg: TrainConfig) -> TrainResult:
    _seed_everything(cfg.seed)
    cfg.out_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device(cfg.device)
    _, train_subset, val_subset = make_split(
        cfg.data_dir,
        val_fraction=cfg.val_fraction,
        seed=cfg.seed,
    )
    train_loader, val_loader = make_dataloaders(
        train_subset,
        val_subset,
        batch_size=cfg.batch_size,
        num_workers=cfg.num_workers,
        seed=cfg.seed,
    )

    model = MelodyClassifier().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=cfg.learning_rate,
        weight_decay=cfg.weight_decay,
    )
    loss_fn = nn.CrossEntropyLoss()

    history: list[dict[str, float]] = []
    best_val_acc = 0.0
    final_train_loss = 0.0
    final_val_acc = 0.0
    checkpoint_path = cfg.out_dir / "best.pt"

    for epoch in range(cfg.epochs):
        model.train()
        running_loss = 0.0
        running_samples = 0
        for images, labels in train_loader:
            images = images.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)
            optimizer.zero_grad()
            logits = model(images)
            loss = loss_fn(logits, labels)
            loss.backward()
            optimizer.step()
            running_loss += float(loss.item()) * images.shape[0]
            running_samples += images.shape[0]

        train_loss = running_loss / max(running_samples, 1)
        val_loss, val_acc = _evaluate(model, val_loader, device, loss_fn)

        history.append(
            {
                "epoch": epoch + 1,
                "train_loss": float(train_loss),
                "val_loss": float(val_loss),
                "val_accuracy": float(val_acc),
            }
        )

        if val_acc >= best_val_acc:
            best_val_acc = val_acc
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "epoch": epoch + 1,
                    "val_accuracy": val_acc,
                    "config": _config_to_jsonable(cfg),
                },
                checkpoint_path,
            )

        final_train_loss = train_loss
        final_val_acc = val_acc

    history_path = cfg.out_dir / "history.json"
    history_path.write_text(json.dumps(history, indent=2) + "\n")

    return TrainResult(
        best_val_accuracy=float(best_val_acc),
        final_val_accuracy=float(final_val_acc),
        final_train_loss=float(final_train_loss),
        epochs_run=cfg.epochs,
        checkpoint_path=checkpoint_path,
        history=history,
    )


def _config_to_jsonable(cfg: TrainConfig) -> dict[str, Any]:
    out = asdict(cfg)
    out["data_dir"] = str(out["data_dir"])
    out["out_dir"] = str(out["out_dir"])
    return out


# --------------------------------------------------------- CLI entry-point ---


@click.command()
@click.option(
    "--data",
    "data_dir",
    type=click.Path(exists=True, file_okay=False, path_type=pathlib.Path),
    required=True,
    help="Dataset directory (folder-of-folders from melody_train.gen).",
)
@click.option(
    "--out",
    "out_dir",
    type=click.Path(file_okay=False, path_type=pathlib.Path),
    required=True,
    help="Output directory for checkpoints + history.",
)
@click.option("--epochs", type=int, default=10, show_default=True)
@click.option("--batch-size", type=int, default=64, show_default=True)
@click.option("--lr", "learning_rate", type=float, default=1e-3, show_default=True)
@click.option("--weight-decay", type=float, default=1e-4, show_default=True)
@click.option("--val-fraction", type=float, default=0.1, show_default=True)
@click.option("--seed", type=int, default=0, show_default=True)
@click.option("--num-workers", type=int, default=0, show_default=True)
@click.option("--device", type=str, default="cpu", show_default=True)
def _cli(**kwargs: Any) -> None:
    cfg = TrainConfig(**kwargs)
    result = train(cfg)
    click.echo(
        f"epochs_run={result.epochs_run} "
        f"best_val_acc={result.best_val_accuracy:.4f} "
        f"final_val_acc={result.final_val_accuracy:.4f} "
        f"final_train_loss={result.final_train_loss:.4f}"
    )
    click.echo(f"checkpoint -> {result.checkpoint_path}")


def main(argv: list[str] | None = None) -> int:
    """CLI entry point. Returns process exit code."""
    try:
        _cli.main(args=argv, standalone_mode=False)
    except click.exceptions.UsageError as exc:
        click.echo(f"Error: {exc.format_message()}", err=True)
        return 2
    except click.exceptions.Abort:
        return 130
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
