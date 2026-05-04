"""PyTorch Dataset wrapping the folder-of-folders layout from ``melody_train.gen``.

Loads PNGs as uint8 tensors of shape ``(1, image_size, image_size)``.
Provides a deterministic stratified train/val split that does not
perturb existing class indices when classes are added.
"""

from __future__ import annotations

import pathlib

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset, Subset

from melody_train.gen import CLASSES


class MelodyROIDataset(Dataset[tuple[torch.Tensor, int]]):
    """Folder-of-folders grayscale ROI dataset.

    Expects ``root/<class>/NNNNNN.png`` for each ``class`` in
    :data:`melody_train.gen.CLASSES`. Files are discovered at
    construction time and indexed by sorted (class, filename) order so
    iteration is deterministic.
    """

    def __init__(
        self,
        root: pathlib.Path,
        *,
        image_size: int = 32,
    ) -> None:
        super().__init__()
        self.root = pathlib.Path(root)
        self.image_size = image_size

        self._items: list[tuple[pathlib.Path, int]] = []
        for cls_idx, cls_name in enumerate(CLASSES):
            cls_dir = self.root / cls_name
            if not cls_dir.is_dir():
                raise FileNotFoundError(
                    f"Class directory {cls_dir} missing — generate dataset first "
                    f"with `python -m melody_train.gen --seed N --out {self.root}`."
                )
            for png in sorted(cls_dir.glob("*.png")):
                self._items.append((png, cls_idx))

        if not self._items:
            raise ValueError(f"No PNGs found under {self.root}")

    def __len__(self) -> int:
        return len(self._items)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, int]:
        path, label = self._items[index]
        with Image.open(path) as img:
            if img.mode != "L":
                img = img.convert("L")
            # np.array(...) copies — np.asarray returns a read-only view
            # which torch.from_numpy can't safely back.
            arr = np.array(img, dtype=np.uint8)
        if arr.shape != (self.image_size, self.image_size):
            raise ValueError(
                f"{path} has shape {arr.shape}, expected ({self.image_size}, {self.image_size})"
            )
        # (H, W) -> (1, H, W)
        tensor = torch.from_numpy(arr).unsqueeze(0)
        return tensor, label

    def class_indices(self) -> list[int]:
        """Per-item class indices (helper for stratified split)."""
        return [label for _, label in self._items]


def make_split(
    root: pathlib.Path,
    *,
    val_fraction: float = 0.1,
    seed: int = 0,
    image_size: int = 32,
) -> tuple[MelodyROIDataset, Subset[tuple[torch.Tensor, int]], Subset[tuple[torch.Tensor, int]]]:
    """Build a stratified train/val split.

    Each class contributes ``val_fraction`` of its samples to val, the
    rest to train. The split is deterministic given ``seed``.

    Returns ``(full_dataset, train_subset, val_subset)``.
    """
    if not 0.0 <= val_fraction <= 1.0:
        raise ValueError(f"val_fraction must be in [0, 1], got {val_fraction}")

    full = MelodyROIDataset(root, image_size=image_size)
    rng = np.random.default_rng(seed)

    train_idx: list[int] = []
    val_idx: list[int] = []
    labels = np.asarray(full.class_indices())
    for cls_idx in range(len(CLASSES)):
        cls_indices = np.flatnonzero(labels == cls_idx)
        # Deterministic shuffle per class
        cls_indices = cls_indices.copy()
        rng.shuffle(cls_indices)
        n_total = len(cls_indices)
        if val_fraction == 0.0:
            n_val = 0
        elif val_fraction == 1.0:
            n_val = n_total
        else:
            # At least one sample to each side so neither split is empty
            # for a class that has any samples.
            n_val = max(1, min(n_total - 1, int(round(n_total * val_fraction))))
        val_idx.extend(cls_indices[:n_val].tolist())
        train_idx.extend(cls_indices[n_val:].tolist())

    return full, Subset(full, sorted(train_idx)), Subset(full, sorted(val_idx))


def make_dataloaders(
    train_subset: Subset[tuple[torch.Tensor, int]],
    val_subset: Subset[tuple[torch.Tensor, int]],
    *,
    batch_size: int = 64,
    num_workers: int = 0,
    seed: int = 0,
) -> tuple[DataLoader, DataLoader]:
    """Build train + val DataLoaders.

    Train shuffles deterministically via a ``torch.Generator`` seeded
    from ``seed``. Val never shuffles.
    """
    train_gen = torch.Generator()
    train_gen.manual_seed(seed)

    train_loader = DataLoader(
        train_subset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        generator=train_gen,
        drop_last=False,
    )
    val_loader = DataLoader(
        val_subset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        drop_last=False,
    )
    return train_loader, val_loader
