"""Tests for the melody-detector MelodyClassifier model.

Tests the small depthwise-separable CNN for 32x32 grayscale ROI classification.
"""

import sys
from pathlib import Path

import pytest
import torch
import torch.nn as nn

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))


class TestNumClassesConstant:
    """Test NUM_CLASSES constant definition."""

    def test_num_classes_matches_gen_classes(self):
        """NUM_CLASSES equals len(CLASSES) == 5."""
        from melody_train.gen import CLASSES
        from melody_train.model import NUM_CLASSES

        assert len(CLASSES) == NUM_CLASSES
        assert NUM_CLASSES == 5


class TestInputSizeConstant:
    """Test INPUT_SIZE constant definition."""

    def test_input_size_is_32(self):
        """INPUT_SIZE == 32 for 32x32 square ROI."""
        from melody_train.model import INPUT_SIZE

        assert INPUT_SIZE == 32


class TestInputChannelsConstant:
    """Test INPUT_CHANNELS constant definition."""

    def test_input_channels_is_1(self):
        """INPUT_CHANNELS == 1 for grayscale."""
        from melody_train.model import INPUT_CHANNELS

        assert INPUT_CHANNELS == 1


class TestMelodyClassifierInstantiation:
    """Test MelodyClassifier constructor and initialization."""

    def test_instantiate_with_defaults(self):
        """MelodyClassifier() instantiates with default num_classes."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()
        assert isinstance(model, nn.Module)
        assert model is not None

    def test_instantiate_with_custom_num_classes(self):
        """MelodyClassifier(num_classes=10) instantiates with custom value."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier(num_classes=10)
        assert isinstance(model, nn.Module)

    def test_default_num_classes_matches_constant(self):
        """MelodyClassifier() defaults to NUM_CLASSES."""
        from melody_train.model import NUM_CLASSES, MelodyClassifier

        model_default = MelodyClassifier()
        model_explicit = MelodyClassifier(num_classes=NUM_CLASSES)

        # Both should match the constant — final-layer output features
        assert list(model_default.parameters())[-1].shape[0] == NUM_CLASSES
        assert list(model_explicit.parameters())[-1].shape[0] == NUM_CLASSES


class TestForwardPassUint8Input:
    """Test forward pass with uint8 input (synthetic generator output)."""

    @pytest.mark.parametrize("batch_size", [1, 2, 8])
    def test_forward_uint8_input_shape(self, batch_size):
        """Forward pass on (N, 1, 32, 32) uint8 returns (N, 5) float logits."""
        from melody_train.model import NUM_CLASSES, MelodyClassifier

        model = MelodyClassifier()
        x = torch.randint(0, 256, (batch_size, 1, 32, 32), dtype=torch.uint8)

        logits = model(x)

        assert logits.shape == (batch_size, NUM_CLASSES)
        assert logits.dtype == torch.float32

    def test_forward_uint8_handles_min_max_values(self):
        """Forward pass handles uint8 range [0, 255] without errors."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()

        # All zeros (minimum uint8)
        x_min = torch.zeros((2, 1, 32, 32), dtype=torch.uint8)
        logits_min = model(x_min)
        assert logits_min.shape == (2, 5)
        assert torch.all(torch.isfinite(logits_min))

        # All 255 (maximum uint8)
        x_max = torch.full((2, 1, 32, 32), 255, dtype=torch.uint8)
        logits_max = model(x_max)
        assert logits_max.shape == (2, 5)
        assert torch.all(torch.isfinite(logits_max))


class TestForwardPassFloat32Input:
    """Test forward pass with float32 input in normalized range."""

    def test_forward_float32_input_shape(self):
        """Forward pass on (N, 1, 32, 32) float32 in [-1, 1] returns (N, 5)."""
        from melody_train.model import NUM_CLASSES, MelodyClassifier

        model = MelodyClassifier()
        x = torch.randn((4, 1, 32, 32)) * 0.5  # Roughly in [-1, 1]

        logits = model(x)

        assert logits.shape == (4, NUM_CLASSES)
        assert logits.dtype == torch.float32

    def test_forward_float32_normalized_range(self):
        """Forward pass handles float32 in [-1, 1] correctly."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()
        x = torch.linspace(-1, 1, 1024).reshape(1, 1, 32, 32)

        logits = model(x)

        assert logits.shape == (1, 5)
        assert torch.all(torch.isfinite(logits))


class TestNormalizationConsistency:
    """Test that uint8 and float32 inputs produce equivalent results."""

    def test_forward_normalization_consistency(self):
        """uint8 X and float32 (X - 128) / 128 produce equal logits within 1e-5."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()
        model.eval()

        # Create a fixed uint8 input covering the full byte range
        torch.manual_seed(0)
        uint8_input = torch.randint(0, 256, (2, 1, 32, 32), dtype=torch.uint8)

        # Manually normalize to float32
        float32_input = (uint8_input.float() - 128.0) / 128.0

        with torch.no_grad():
            logits_uint8 = model(uint8_input)
            logits_float32 = model(float32_input)

        # Should be nearly identical (allowing for normalization in uint8 path)
        # The test passes if they're close, indicating consistent normalization
        assert logits_uint8.shape == logits_float32.shape
        assert torch.allclose(logits_uint8, logits_float32, atol=1e-5)


class TestForwardPassNumericalStability:
    """Test numerical stability of forward pass."""

    def test_forward_logits_are_finite(self):
        """Forward pass never produces NaN or Inf for random uint8 input."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()

        for _ in range(10):
            x = torch.randint(0, 256, (4, 1, 32, 32), dtype=torch.uint8)
            logits = model(x)

            assert torch.all(torch.isfinite(logits)), f"Non-finite logits detected: {logits}"


class TestModelSize:
    """Test model parameter count and estimated quantized size."""

    def test_parameter_count_under_budget(self):
        """Total trainable parameters < 50,000."""
        from melody_train.model import MelodyClassifier, parameter_count

        model = MelodyClassifier()
        count = parameter_count(model)

        assert isinstance(count, int)
        assert count > 0
        assert count < 50_000, f"Parameter count {count} exceeds budget of 50,000"

    def test_estimated_int8_size_under_budget(self):
        """Estimated INT8 quantized size < 200,000 bytes (200 KB)."""
        from melody_train.model import MelodyClassifier, estimated_int8_size_bytes

        model = MelodyClassifier()
        size_bytes = estimated_int8_size_bytes(model)

        assert isinstance(size_bytes, int)
        assert size_bytes > 0
        assert size_bytes < 200_000, (
            f"Estimated INT8 size {size_bytes} exceeds budget of 200,000 bytes"
        )


class TestModelTrainability:
    """Test that model parameters are trainable."""

    def test_model_is_trainable(self):
        """All parameters have requires_grad=True; backward pass produces gradients."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()

        # Check all parameters require grad
        for name, param in model.named_parameters():
            assert param.requires_grad, f"Parameter {name} has requires_grad=False"

        # Run forward, loss, backward
        x = torch.randint(0, 256, (2, 1, 32, 32), dtype=torch.uint8)
        logits = model(x)

        # Dummy loss (just sum for testing)
        loss = logits.sum()
        loss.backward()

        # Check at least one parameter has gradients
        has_gradients = False
        for param in model.parameters():
            if param.grad is not None and torch.any(param.grad != 0):
                has_gradients = True
                break

        assert has_gradients, "No parameters received non-zero gradients"


class TestModelEvalMode:
    """Test model in evaluation mode."""

    def test_model_eval_mode_is_deterministic(self):
        """Forward pass in .eval() mode is deterministic (bit-identical outputs)."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()
        model.eval()

        x = torch.randint(0, 256, (2, 1, 32, 32), dtype=torch.uint8)

        with torch.no_grad():
            logits_1 = model(x)
            logits_2 = model(x)

        assert torch.equal(logits_1, logits_2), "Forward passes in eval mode are not bit-identical"

    def test_model_train_mode_exists(self):
        """Model can be set to train mode without errors."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()
        model.train()

        x = torch.randint(0, 256, (1, 1, 32, 32), dtype=torch.uint8)
        logits = model(x)

        assert logits.shape == (1, 5)
        assert torch.all(torch.isfinite(logits))


class TestArchitecture:
    """Test model architecture properties."""

    def test_has_convolutional_layers(self):
        """Model contains Conv2d layers."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()

        has_conv = any(isinstance(m, nn.Conv2d) for m in model.modules())
        assert has_conv, "Model has no Conv2d layers"

    def test_has_pooling_layers(self):
        """Model contains pooling layers (MaxPool2d or AvgPool2d)."""
        from melody_train.model import MelodyClassifier

        model = MelodyClassifier()

        has_pool = any(isinstance(m, (nn.MaxPool2d, nn.AvgPool2d)) for m in model.modules())
        assert has_pool, "Model has no pooling layers"

    def test_has_final_linear_layer(self):
        """Model's final layer is Linear with output size == NUM_CLASSES."""
        from melody_train.model import NUM_CLASSES, MelodyClassifier

        model = MelodyClassifier()

        # Get the last Linear layer
        linear_layers = [m for m in model.modules() if isinstance(m, nn.Linear)]
        assert len(linear_layers) > 0, "Model has no Linear layers"

        final_linear = linear_layers[-1]
        assert final_linear.out_features == NUM_CLASSES
