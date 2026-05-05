"""Tests for the melody-detector MelodyROIDataset and utilities.

Tests the dataset loader for folder-of-folders structure and train/val splitting.
"""

import sys
from pathlib import Path

import pytest
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))


@pytest.fixture(scope="session")
def synth_data_dir(tmp_path_factory):
    """Generate a small synthetic dataset once per test session."""
    from melody_train.gen import generate_dataset

    data_dir = tmp_path_factory.mktemp("synth_data")
    generate_dataset(seed=0, out_dir=data_dir, per_class=8, image_size=32)
    return data_dir


class TestMelodyROIDatasetInstantiation:
    """Test MelodyROIDataset constructor."""

    def test_instantiate_with_root(self, synth_data_dir):
        """MelodyROIDataset(root) instantiates successfully."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)
        assert isinstance(dataset, torch.utils.data.Dataset)

    def test_instantiate_with_custom_image_size(self, synth_data_dir):
        """MelodyROIDataset(root, image_size=32) instantiates with custom size."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir, image_size=32)
        assert isinstance(dataset, torch.utils.data.Dataset)


class TestMelodyROIDatasetLength:
    """Test dataset length."""

    def test_dataset_length(self, synth_data_dir):
        """len(dataset) == 5 classes × 8 per_class == 40."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)
        assert len(dataset) == 40


class TestMelodyROIDatasetGetItem:
    """Test dataset __getitem__ method."""

    def test_dataset_getitem_returns_tuple(self, synth_data_dir):
        """dataset[idx] returns a (tensor, int) tuple."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)
        item = dataset[0]

        assert isinstance(item, tuple)
        assert len(item) == 2

    def test_dataset_getitem_shape(self, synth_data_dir):
        """dataset[0] image tensor has shape (1, 32, 32)."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)
        image, label = dataset[0]

        assert image.shape == (1, 32, 32)
        assert isinstance(label, int)

    def test_dataset_getitem_dtype(self, synth_data_dir):
        """Image tensor is torch.uint8."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)
        image, label = dataset[0]

        assert image.dtype == torch.uint8

    def test_dataset_label_range(self, synth_data_dir):
        """All labels are integers in [0, 5)."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)

        for idx in range(len(dataset)):
            image, label = dataset[idx]
            assert isinstance(label, int)
            assert 0 <= label < 5


class TestMelodyROIDatasetBalance:
    """Test dataset class balance."""

    def test_dataset_class_balance(self, synth_data_dir):
        """Each class index appears exactly 8 times (per_class=8)."""
        from melody_train.dataset import MelodyROIDataset

        dataset = MelodyROIDataset(synth_data_dir)

        class_counts = [0] * 5
        for idx in range(len(dataset)):
            _, label = dataset[idx]
            class_counts[label] += 1

        # All classes should have exactly 8 samples
        assert class_counts == [8, 8, 8, 8, 8]


class TestMakeSplit:
    """Test make_split function for train/val splitting."""

    def test_make_split_returns_tuple(self, synth_data_dir):
        """make_split returns (full_dataset, train_subset, val_subset)."""
        from melody_train.dataset import make_split

        result = make_split(synth_data_dir)

        assert isinstance(result, tuple)
        assert len(result) == 3

    def test_make_split_sizes(self, synth_data_dir):
        """Train + val sizes sum to total; val ≈ val_fraction × total."""
        from melody_train.dataset import make_split

        full_dataset, train_subset, val_subset = make_split(synth_data_dir, val_fraction=0.1)

        total_size = len(full_dataset)
        train_size = len(train_subset)
        val_size = len(val_subset)

        assert train_size + val_size == total_size
        # With val_fraction=0.1 and 40 samples, expect roughly 36 train / 4 val
        # Allow ±1 per class for stratification rounding
        assert val_size == pytest.approx(total_size * 0.1, abs=1)

    def test_make_split_disjoint(self, synth_data_dir):
        """Train and val indices are disjoint (no overlap)."""
        from melody_train.dataset import make_split

        _, train_subset, val_subset = make_split(synth_data_dir)

        train_indices = set(train_subset.indices)
        val_indices = set(val_subset.indices)

        assert len(train_indices & val_indices) == 0, (
            "Train and val subsets have overlapping indices"
        )

    def test_make_split_is_deterministic(self, synth_data_dir):
        """Two calls with same seed produce identical train/val indices."""
        from melody_train.dataset import make_split

        _, train_1, val_1 = make_split(synth_data_dir, seed=42)
        _, train_2, val_2 = make_split(synth_data_dir, seed=42)

        assert train_1.indices == train_2.indices
        assert val_1.indices == val_2.indices

    def test_make_split_stratified(self, synth_data_dir):
        """Each class is represented in both splits proportional to val_fraction."""
        from melody_train.dataset import make_split

        full_dataset, train_subset, val_subset = make_split(
            synth_data_dir, val_fraction=0.1, seed=0
        )

        # Count class distribution in train and val
        train_class_counts = [0] * 5
        val_class_counts = [0] * 5

        for idx in train_subset.indices:
            _, label = full_dataset[idx]
            train_class_counts[label] += 1

        for idx in val_subset.indices:
            _, label = full_dataset[idx]
            val_class_counts[label] += 1

        # Each class should be represented in both sets, with val close to
        # val_fraction × per_class within rounding (the spec allows ±1).
        per_class = 8  # matches synth_data_dir fixture
        expected_val = round(per_class * 0.1)
        for cls in range(5):
            assert train_class_counts[cls] > 0, f"Class {cls} missing from train"
            assert val_class_counts[cls] > 0, f"Class {cls} missing from val"
            assert abs(val_class_counts[cls] - expected_val) <= 1, (
                f"Class {cls}: val count {val_class_counts[cls]} too far from {expected_val}"
            )
            assert train_class_counts[cls] + val_class_counts[cls] == per_class, (
                f"Class {cls}: train+val != per_class"
            )


class TestMakeSplitEdgeCases:
    """Test make_split edge cases."""

    def test_make_split_different_seeds_yield_different_splits(self, synth_data_dir):
        """Two calls with different seeds produce different train/val indices."""
        from melody_train.dataset import make_split

        _, train_0, val_0 = make_split(synth_data_dir, seed=0)
        _, train_1, val_1 = make_split(synth_data_dir, seed=1)

        # Indices should differ (not necessarily 100%, but should differ)
        assert train_0.indices != train_1.indices or val_0.indices != val_1.indices

    def test_make_split_val_fraction_zero(self, synth_data_dir):
        """val_fraction=0 produces empty val set and full train set."""
        from melody_train.dataset import make_split

        full_dataset, train_subset, val_subset = make_split(synth_data_dir, val_fraction=0.0)

        assert len(train_subset) == len(full_dataset)
        assert len(val_subset) == 0

    def test_make_split_val_fraction_one(self, synth_data_dir):
        """val_fraction=1.0 produces empty train set and full val set."""
        from melody_train.dataset import make_split

        full_dataset, train_subset, val_subset = make_split(synth_data_dir, val_fraction=1.0)

        assert len(train_subset) == 0
        assert len(val_subset) == len(full_dataset)


class TestMakeDataloaders:
    """Test make_dataloaders function."""

    def test_make_dataloaders_returns_tuple(self, synth_data_dir):
        """make_dataloaders returns (train_loader, val_loader)."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        result = make_dataloaders(train_subset, val_subset)

        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_make_dataloaders_train_loader_type(self, synth_data_dir):
        """Train loader is a DataLoader instance."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        train_loader, _ = make_dataloaders(train_subset, val_subset)

        assert isinstance(train_loader, DataLoader)

    def test_make_dataloaders_val_loader_type(self, synth_data_dir):
        """Val loader is a DataLoader instance."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        _, val_loader = make_dataloaders(train_subset, val_subset)

        assert isinstance(val_loader, DataLoader)

    def test_make_dataloaders_yields_correct_shape(self, synth_data_dir):
        """Loader yields (images, labels) with images shape (batch_size, 1, 32, 32)."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        train_loader, _ = make_dataloaders(train_subset, val_subset, batch_size=4)

        images, labels = next(iter(train_loader))

        assert images.shape == (4, 1, 32, 32) or len(images) < 4
        assert labels.shape[0] == images.shape[0]

    def test_make_dataloaders_last_batch_smaller(self, synth_data_dir):
        """Last batch can be smaller than batch_size if not evenly divisible."""
        from melody_train.dataset import make_dataloaders, make_split

        full_dataset, train_subset, val_subset = make_split(synth_data_dir)
        # With 36 train samples and batch_size=7, last batch should be 1
        train_loader, _ = make_dataloaders(train_subset, val_subset, batch_size=7)

        batches = list(train_loader)
        # Last batch may be smaller
        last_batch_images, _ = batches[-1]
        assert last_batch_images.shape[0] <= 7


class TestDataloaderDeterminism:
    """Test determinism of dataloaders."""

    def test_make_dataloaders_train_shuffles_deterministically(self, synth_data_dir):
        """Two train loaders with same seed yield batches in same order."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset_1, val_subset_1 = make_split(synth_data_dir, seed=0)
        train_loader_1, _ = make_dataloaders(train_subset_1, val_subset_1, seed=0, batch_size=4)

        _, train_subset_2, val_subset_2 = make_split(synth_data_dir, seed=0)
        train_loader_2, _ = make_dataloaders(train_subset_2, val_subset_2, seed=0, batch_size=4)

        for batch_1, batch_2 in zip(train_loader_1, train_loader_2, strict=True):
            images_1, labels_1 = batch_1
            images_2, labels_2 = batch_2

            assert torch.equal(images_1, images_2)
            assert torch.equal(labels_1, labels_2)

    def test_make_dataloaders_different_seeds_shuffle_differently(self, synth_data_dir):
        """Train loaders with different seeds yield different batch orderings."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset_0, val_subset_0 = make_split(synth_data_dir, seed=0)
        train_loader_0, _ = make_dataloaders(train_subset_0, val_subset_0, seed=0, batch_size=4)

        _, train_subset_1, val_subset_1 = make_split(synth_data_dir, seed=0)
        train_loader_1, _ = make_dataloaders(train_subset_1, val_subset_1, seed=1, batch_size=4)

        # Collect all batch indices to check if ordering differs
        indices_0 = []
        indices_1 = []

        for batch in train_loader_0:
            images, _ = batch
            # We can't directly get indices, but we can check if outputs differ
            indices_0.append(images[0].sum().item())

        for batch in train_loader_1:
            images, _ = batch
            indices_1.append(images[0].sum().item())

        # Sequences should differ (at least some batches in different order)
        # This is a relaxed check — if seeds differ, order should differ
        assert indices_0 != indices_1

    def test_make_dataloaders_val_does_not_shuffle(self, synth_data_dir):
        """Val loader does not shuffle; same order on repeated creation."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset_1, val_subset_1 = make_split(synth_data_dir, seed=0)
        _, val_loader_1 = make_dataloaders(train_subset_1, val_subset_1, seed=0, batch_size=4)

        _, train_subset_2, val_subset_2 = make_split(synth_data_dir, seed=0)
        _, val_loader_2 = make_dataloaders(train_subset_2, val_subset_2, seed=1, batch_size=4)

        for batch_1, batch_2 in zip(val_loader_1, val_loader_2, strict=True):
            images_1, labels_1 = batch_1
            images_2, labels_2 = batch_2

            assert torch.equal(images_1, images_2), (
                "Val loader shuffled or differed despite not shuffling"
            )
            assert torch.equal(labels_1, labels_2)


class TestDataloaderBatchSize:
    """Test batch size configuration."""

    @pytest.mark.parametrize("batch_size", [1, 8, 16, 32])
    def test_make_dataloaders_respects_batch_size(self, synth_data_dir, batch_size):
        """DataLoader respects batch_size parameter."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        train_loader, _ = make_dataloaders(train_subset, val_subset, batch_size=batch_size)

        for images, labels in train_loader:
            # All batches except possibly the last should match batch_size
            assert images.shape[0] <= batch_size
            assert images.shape[0] == labels.shape[0]

            # Check shape is correct
            assert images.shape[1:] == (1, 32, 32)

            break  # Just check first batch


class TestDataloaderNumWorkers:
    """Test num_workers configuration."""

    def test_make_dataloaders_accepts_num_workers(self, synth_data_dir):
        """make_dataloaders accepts num_workers parameter."""
        from melody_train.dataset import make_dataloaders, make_split

        _, train_subset, val_subset = make_split(synth_data_dir)
        train_loader, _ = make_dataloaders(train_subset, val_subset, num_workers=0)

        # Just verify it doesn't crash
        batch = next(iter(train_loader))
        assert batch is not None
