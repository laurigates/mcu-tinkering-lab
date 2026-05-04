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
