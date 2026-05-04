"""Tests for the melody-detector ROI generator module.

Tests the synthetic dataset generation for the 5-class melody note classifier:
empty, quarter, half, rest, other.
"""

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

# Import will be available once gen.py is implemented
# from melody_train.gen import CLASSES, generate_dataset, main, render_class_sample


class TestClassesConstant:
    """Test the CLASSES constant definition."""

    def test_classes_constant_shape(self):
        """CLASSES is a non-empty tuple containing the 5 expected class names."""
        from melody_train.gen import CLASSES

        assert isinstance(CLASSES, tuple)
        assert len(CLASSES) == 5
        assert all(isinstance(cls, str) for cls in CLASSES)
        assert CLASSES == ("empty", "quarter", "half", "rest", "other")


class TestRenderClassSample:
    """Test render_class_sample function for single ROI generation."""

    @pytest.mark.parametrize("cls", ["empty", "quarter", "half", "rest", "other"])
    def test_render_returns_correct_shape_and_dtype(self, cls, make_rng):
        """render_class_sample returns ndarray with shape (32, 32) and dtype uint8."""
        from melody_train.gen import render_class_sample

        rng = make_rng(42)
        result = render_class_sample(cls, rng)

        assert isinstance(result, np.ndarray)
        assert result.shape == (32, 32)
        assert result.dtype == np.uint8

    @pytest.mark.parametrize("image_size", [16, 32, 64])
    @pytest.mark.parametrize("cls", ["empty", "quarter"])
    def test_render_respects_image_size(self, cls, image_size, make_rng):
        """Output shape matches the specified image_size parameter."""
        from melody_train.gen import render_class_sample

        rng = make_rng(42)
        result = render_class_sample(cls, rng, image_size=image_size)

        assert result.shape == (image_size, image_size)
        assert result.dtype == np.uint8

    def test_render_invalid_class_raises(self, make_rng):
        """Passing an unknown class string raises ValueError."""
        from melody_train.gen import render_class_sample

        rng = make_rng(42)

        with pytest.raises(ValueError):
            render_class_sample("invalid_class", rng)

    @pytest.mark.parametrize("cls", ["empty", "quarter", "half", "rest", "other"])
    def test_render_is_deterministic_per_rng(self, cls, make_rng):
        """Two calls with RNGs seeded identically produce bit-identical output."""
        from melody_train.gen import render_class_sample

        rng1 = make_rng(42)
        result1 = render_class_sample(cls, rng1)

        rng2 = make_rng(42)
        result2 = render_class_sample(cls, rng2)

        assert np.array_equal(result1, result2)

    @pytest.mark.parametrize("cls", ["empty", "quarter", "half", "rest", "other"])
    def test_render_differs_across_seeds(self, cls, make_rng):
        """Two calls with different seeds produce different output (non-identical)."""
        from melody_train.gen import render_class_sample

        rng1 = make_rng(42)
        result1 = render_class_sample(cls, rng1)

        rng2 = make_rng(99)
        result2 = render_class_sample(cls, rng2)

        # Outputs should differ; at least 5 pixels should be different
        diff_count = np.sum(result1 != result2)
        assert diff_count >= 5, f"Expected >= 5 differing pixels, got {diff_count}"

    @pytest.mark.parametrize("cls", ["quarter", "half", "rest", "other"])
    def test_render_uses_full_grayscale_range_non_empty(self, cls, make_rng):
        """Non-empty classes contain both ink-dark (< 128) and paper-light (> 128) pixels."""
        from melody_train.gen import render_class_sample

        rng = make_rng(42)
        result = render_class_sample(cls, rng)

        has_dark = np.any(result < 128)
        has_light = np.any(result > 128)

        assert has_dark, f"Class {cls} missing dark pixels (ink)"
        assert has_light, f"Class {cls} missing light pixels (paper)"

    def test_render_empty_is_mostly_bright(self, make_rng):
        """Empty class output is mostly paper-white (mean > 200)."""
        from melody_train.gen import render_class_sample

        rng = make_rng(42)
        result = render_class_sample("empty", rng)

        assert np.mean(result) > 200

    @pytest.mark.parametrize("canonical_cls", ["quarter", "half", "rest"])
    def test_render_other_is_visually_distinct_from_canonical(self, canonical_cls, make_rng):
        """Other class output is not pixel-equal to canonical noteheads with same seed."""
        from melody_train.gen import render_class_sample

        rng1 = make_rng(42)
        other = render_class_sample("other", rng1)

        rng2 = make_rng(42)
        canonical = render_class_sample(canonical_cls, rng2)

        assert not np.array_equal(other, canonical), (
            f"'other' is identical to '{canonical_cls}' with same seed"
        )

    @pytest.mark.parametrize("cls", ["empty", "quarter", "half", "rest", "other"])
    def test_render_does_not_use_global_random_state(self, cls, make_rng):
        """Only the passed rng matters; np.random.seed has no effect."""
        from melody_train.gen import render_class_sample

        # Set global seed and render
        np.random.seed(42)
        rng_a = make_rng(100)
        result_a = render_class_sample(cls, rng_a)

        # Set global seed again and render with different rng seed
        np.random.seed(42)
        rng_b = make_rng(200)
        result_b = render_class_sample(cls, rng_b)

        # Results should differ because rng seeds differ
        diff_count = np.sum(result_a != result_b)
        assert diff_count >= 5, f"Global seed affected output; only {diff_count} pixels differ"


class TestGenerateDataset:
    """Test generate_dataset function for batch ROI generation."""

    def test_generate_dataset_writes_expected_file_count(self, tmp_path):
        """Generated dataset has per_class files in class-named subdirectories."""
        from melody_train.gen import generate_dataset

        per_class = 10
        generate_dataset(seed=0, out_dir=tmp_path, per_class=per_class, image_size=32)

        # Each class should have exactly per_class PNG files
        for cls in ["empty", "quarter", "half", "rest", "other"]:
            class_dir = tmp_path / cls
            assert class_dir.exists(), f"Class directory {cls} not created"
            pngs = list(class_dir.glob("*.png"))
            assert len(pngs) == per_class, f"Class {cls}: expected {per_class}, got {len(pngs)}"

    def test_generate_dataset_file_naming_convention(self, tmp_path):
        """Generated PNG files use 6-digit zero-padded indices (000000.png, etc.)."""
        from melody_train.gen import generate_dataset

        per_class = 3
        generate_dataset(seed=0, out_dir=tmp_path, per_class=per_class)

        # Check naming convention
        class_dir = tmp_path / "empty"
        pngs = sorted(class_dir.glob("*.png"))
        expected_names = ["000000.png", "000001.png", "000002.png"]
        actual_names = [png.name for png in pngs]
        assert actual_names == expected_names

    def test_generate_dataset_returns_correct_counts(self, tmp_path):
        """Return value maps each class to per_class count."""
        from melody_train.gen import CLASSES, generate_dataset

        per_class = 7
        result = generate_dataset(seed=0, out_dir=tmp_path, per_class=per_class)

        assert isinstance(result, dict)
        assert set(result.keys()) == set(CLASSES)
        for cls in CLASSES:
            assert result[cls] == per_class

    def test_generate_dataset_writes_manifest(self, tmp_path):
        """manifest.json is written with correct schema and types."""
        from melody_train.gen import CLASSES, generate_dataset

        per_class = 5
        image_size = 32
        seed = 42

        generate_dataset(
            seed=seed,
            out_dir=tmp_path,
            per_class=per_class,
            image_size=image_size,
        )

        manifest_path = tmp_path / "manifest.json"
        assert manifest_path.exists()

        with open(manifest_path) as f:
            manifest = json.load(f)

        # Check schema
        assert "seed" in manifest
        assert "per_class" in manifest
        assert "image_size" in manifest
        assert "classes" in manifest
        assert "counts" in manifest
        assert "version" in manifest

        # Check types and values
        assert isinstance(manifest["seed"], int)
        assert manifest["seed"] == seed
        assert isinstance(manifest["per_class"], int)
        assert manifest["per_class"] == per_class
        assert isinstance(manifest["image_size"], int)
        assert manifest["image_size"] == image_size
        assert isinstance(manifest["classes"], list)
        assert manifest["classes"] == list(CLASSES)
        assert isinstance(manifest["counts"], dict)
        assert manifest["counts"] == dict.fromkeys(CLASSES, per_class)
        assert isinstance(manifest["version"], str)

    def test_generate_dataset_is_reproducible_from_seed(self, tmp_path):
        """Running with same seed produces byte-identical PNG files."""
        from melody_train.gen import CLASSES, generate_dataset

        tmp_path_1 = tmp_path / "run1"
        tmp_path_2 = tmp_path / "run2"
        tmp_path_1.mkdir(parents=True)
        tmp_path_2.mkdir(parents=True)

        # Generate with same seed in two different directories
        generate_dataset(seed=42, out_dir=tmp_path_1, per_class=5, image_size=32)
        generate_dataset(seed=42, out_dir=tmp_path_2, per_class=5, image_size=32)

        # Compare SHA256 hashes of corresponding PNG files
        for cls in CLASSES:
            for idx in range(5):
                filename = f"{idx:06d}.png"
                path_1 = tmp_path_1 / cls / filename
                path_2 = tmp_path_2 / cls / filename

                hash_1 = hashlib.sha256(path_1.read_bytes()).hexdigest()
                hash_2 = hashlib.sha256(path_2.read_bytes()).hexdigest()

                assert hash_1 == hash_2, f"{cls}/{filename} hashes differ across runs"

    @pytest.mark.parametrize("cls", ["empty", "quarter", "half", "rest", "other"])
    def test_generate_dataset_differs_with_different_seeds(self, tmp_path, cls):
        """Running with different seeds produces at least one differing PNG per class."""
        from melody_train.gen import generate_dataset

        tmp_path_seed0 = tmp_path / "seed0"
        tmp_path_seed1 = tmp_path / "seed1"
        tmp_path_seed0.mkdir(parents=True)
        tmp_path_seed1.mkdir(parents=True)

        generate_dataset(seed=0, out_dir=tmp_path_seed0, per_class=3, image_size=32)
        generate_dataset(seed=1, out_dir=tmp_path_seed1, per_class=3, image_size=32)

        # At least one file should differ
        files_differ = False
        for idx in range(3):
            filename = f"{idx:06d}.png"
            path_0 = tmp_path_seed0 / cls / filename
            path_1 = tmp_path_seed1 / cls / filename

            hash_0 = hashlib.sha256(path_0.read_bytes()).hexdigest()
            hash_1 = hashlib.sha256(path_1.read_bytes()).hexdigest()

            if hash_0 != hash_1:
                files_differ = True
                break

        assert files_differ, f"No files differ for class {cls} between seed=0 and seed=1"

    def test_pngs_are_valid_grayscale_images(self, tmp_path):
        """Generated PNG files are valid grayscale images with correct size."""
        from melody_train.gen import CLASSES, generate_dataset

        image_size = 32
        generate_dataset(seed=0, out_dir=tmp_path, per_class=2, image_size=image_size)

        # Sample one PNG per class
        for cls in CLASSES:
            png_path = tmp_path / cls / "000000.png"
            assert png_path.exists()

            img = Image.open(png_path)
            # Mode should be L (8-bit grayscale) or 1 (1-bit black/white)
            assert img.mode in ("L", "1"), f"Unexpected mode {img.mode} for {cls}"
            assert img.size == (image_size, image_size), f"Unexpected size {img.size}"


class TestMainCLI:
    """Test main CLI entry point."""

    def test_main_cli_smoke(self, tmp_path):
        """Invoking main CLI with valid args returns 0 and populates output dir."""
        from melody_train.gen import main

        result = main(
            [
                "--seed",
                "0",
                "--out",
                str(tmp_path),
                "--per-class",
                "5",
            ]
        )

        assert result == 0
        assert (tmp_path / "empty").exists()
        assert (tmp_path / "quarter").exists()
        assert (tmp_path / "manifest.json").exists()

    def test_main_cli_missing_required_args(self):
        """Invoking main with missing required args returns nonzero exit code."""
        from melody_train.gen import main

        result = main([])
        assert result != 0

    def test_main_cli_missing_seed(self, tmp_path):
        """Invoking main without --seed returns nonzero exit code."""
        from melody_train.gen import main

        result = main(["--out", str(tmp_path)])
        assert result != 0

    def test_main_cli_missing_out(self):
        """Invoking main without --out returns nonzero exit code."""
        from melody_train.gen import main

        result = main(["--seed", "0"])
        assert result != 0

    def test_main_cli_with_defaults(self, tmp_path):
        """Invoking main with defaults uses per_class=5000 and image_size=32."""
        import json

        from melody_train.gen import main

        result = main(["--seed", "99", "--out", str(tmp_path)])

        assert result == 0
        manifest_path = tmp_path / "manifest.json"
        with open(manifest_path) as f:
            manifest = json.load(f)

        assert manifest["per_class"] == 5000
        assert manifest["image_size"] == 32
