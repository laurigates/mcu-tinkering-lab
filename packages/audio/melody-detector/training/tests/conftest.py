"""Shared test fixtures for melody-detector training tests."""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add src directory to path so all test modules can import project code
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))


@pytest.fixture
def make_rng():
    """Factory fixture for creating deterministic numpy Generators from a seed."""

    def _factory(seed: int) -> np.random.Generator:
        return np.random.default_rng(seed)

    return _factory


@pytest.fixture(scope="session")
def synth_data_dir(tmp_path_factory):
    """Generate a small synthetic dataset once per test session.

    Generates 8 samples per class (40 total) for fast dataset/train tests.
    """
    from melody_train.gen import generate_dataset

    data_dir = tmp_path_factory.mktemp("synth_data")
    generate_dataset(seed=0, out_dir=data_dir, per_class=8, image_size=32)
    return data_dir
