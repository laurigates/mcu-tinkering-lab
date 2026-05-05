"""Tests for the melody-detector ONNX export pipeline.

Covers:
- ``ExportConfig`` / ``ExportResult`` dataclass shape
- ``export()`` produces a loadable ONNX file at the requested opset
- BatchNorm is folded into the preceding Conv (no standalone
  ``BatchNormalization`` op in the graph)
- The normalization Sub/Div ops appear in the graph (so ESP-PPQ has
  something to fold into the first Conv weights)
- Forward parity between the PyTorch model and the exported ONNX (only
  runs when ``onnxruntime`` is installed; ``[quantize]`` extra brings
  it in)
- CLI surface — ``main([...])`` returns 0 on success, nonzero on bad args
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))


# ----------------------------------------------------------------- helpers ----


def _save_dummy_checkpoint(out_dir: Path, seed: int = 0) -> Path:
    """Save an untrained MelodyClassifier checkpoint in train.py's format."""
    from melody_train.model import MelodyClassifier

    torch.manual_seed(seed)
    model = MelodyClassifier()
    out_path = out_dir / "best.pt"
    torch.save(
        {
            "model_state": model.state_dict(),
            "epoch": 1,
            "val_accuracy": 0.5,
            "config": {},
        },
        out_path,
    )
    return out_path


@pytest.fixture
def dummy_checkpoint(tmp_path: Path) -> Path:
    return _save_dummy_checkpoint(tmp_path, seed=0)


@pytest.fixture
def exported_onnx(tmp_path: Path, dummy_checkpoint: Path) -> Path:
    """Export the dummy checkpoint to ONNX and return the path."""
    from melody_train.export import ExportConfig, export

    out = tmp_path / "model.onnx"
    cfg = ExportConfig(checkpoint_path=dummy_checkpoint, out_path=out)
    export(cfg)
    return out


# ------------------------------------------------------- ExportConfig shape ----


class TestExportConfig:
    def test_instantiate_with_required_fields(self, tmp_path):
        from melody_train.export import ExportConfig

        cfg = ExportConfig(
            checkpoint_path=tmp_path / "best.pt",
            out_path=tmp_path / "model.onnx",
        )
        assert cfg.checkpoint_path == tmp_path / "best.pt"
        assert cfg.out_path == tmp_path / "model.onnx"

    def test_defaults(self, tmp_path):
        from melody_train.export import ExportConfig

        cfg = ExportConfig(
            checkpoint_path=tmp_path / "best.pt",
            out_path=tmp_path / "model.onnx",
        )
        assert cfg.opset_version == 17
        assert cfg.input_name == "input"
        assert cfg.output_name == "output"

    def test_is_frozen(self, tmp_path):
        from melody_train.export import ExportConfig

        cfg = ExportConfig(
            checkpoint_path=tmp_path / "best.pt",
            out_path=tmp_path / "model.onnx",
        )
        with pytest.raises((AttributeError, TypeError)):
            cfg.opset_version = 18  # type: ignore[misc]


class TestExportResult:
    def test_instantiate(self, tmp_path):
        from melody_train.export import ExportResult

        result = ExportResult(
            onnx_path=tmp_path / "model.onnx",
            opset_version=17,
            input_name="input",
            output_name="output",
        )
        assert result.onnx_path == tmp_path / "model.onnx"
        assert result.opset_version == 17

    def test_is_frozen(self, tmp_path):
        from melody_train.export import ExportResult

        result = ExportResult(
            onnx_path=tmp_path / "model.onnx",
            opset_version=17,
            input_name="input",
            output_name="output",
        )
        with pytest.raises((AttributeError, TypeError)):
            result.opset_version = 18  # type: ignore[misc]


# ----------------------------------------------------- export() behaviour ----


class TestExportFile:
    def test_export_creates_file(self, tmp_path, dummy_checkpoint):
        from melody_train.export import ExportConfig, export

        out = tmp_path / "model.onnx"
        result = export(ExportConfig(checkpoint_path=dummy_checkpoint, out_path=out))

        assert out.exists()
        assert out.stat().st_size > 0
        assert result.onnx_path == out

    def test_export_creates_parent_dirs(self, tmp_path, dummy_checkpoint):
        from melody_train.export import ExportConfig, export

        out = tmp_path / "nested" / "subdir" / "model.onnx"
        export(ExportConfig(checkpoint_path=dummy_checkpoint, out_path=out))

        assert out.exists()

    def test_export_size_under_50kb(self, exported_onnx):
        # MelodyClassifier is ~2.5K params (~10 KB float32 ONNX), well under 50 KB.
        # If this trips, something has gone very wrong (extra weights, debug data).
        assert exported_onnx.stat().st_size < 50_000

    def test_missing_checkpoint_raises(self, tmp_path):
        from melody_train.export import ExportConfig, export

        with pytest.raises(FileNotFoundError):
            export(
                ExportConfig(
                    checkpoint_path=tmp_path / "nope.pt",
                    out_path=tmp_path / "model.onnx",
                )
            )


class TestExportGraphInvariants:
    def test_loadable_via_onnx(self, exported_onnx):
        import onnx

        model = onnx.load(str(exported_onnx))
        onnx.checker.check_model(model)

    def test_opset_version_is_17(self, exported_onnx):
        import onnx

        model = onnx.load(str(exported_onnx))
        # The default ONNX domain ('') carries the opset version.
        default_opsets = [o.version for o in model.opset_import if o.domain in ("", "ai.onnx")]
        assert default_opsets, "no default-domain opset import"
        assert default_opsets[0] == 17, f"expected opset 17, got {default_opsets[0]}"

    def test_no_standalone_batchnorm(self, exported_onnx):
        """EVAL-mode export folds BatchNorm into the preceding Conv."""
        import onnx

        model = onnx.load(str(exported_onnx))
        op_types = {n.op_type for n in model.graph.node}
        assert "BatchNormalization" not in op_types, (
            f"BN should be folded; graph contains: {sorted(op_types)}"
        )

    def test_normalization_sub_div_present(self, exported_onnx):
        """The (x-128)/128 normalization survives as Sub/Div ops in the graph."""
        import onnx

        model = onnx.load(str(exported_onnx))
        op_types = {n.op_type for n in model.graph.node}
        assert "Sub" in op_types, "Sub op missing — normalization not in graph"
        assert "Div" in op_types, "Div op missing — normalization not in graph"

    def test_first_meaningful_op_is_normalization(self, exported_onnx):
        """The first op operating on the input tensor is the Sub of (x-128)."""
        import onnx

        model = onnx.load(str(exported_onnx))
        # Walk the graph topologically starting at "input"; the first node
        # that consumes "input" should be Sub (the normalization), not Conv.
        consumers = [n for n in model.graph.node if "input" in list(n.input)]
        assert consumers, "no node consumes the 'input' tensor"
        assert consumers[0].op_type == "Sub", (
            f"first consumer of 'input' should be Sub (normalization), got {consumers[0].op_type}"
        )

    def test_io_names(self, exported_onnx):
        import onnx

        model = onnx.load(str(exported_onnx))
        assert [i.name for i in model.graph.input] == ["input"]
        assert [o.name for o in model.graph.output] == ["output"]

    def test_input_shape_is_dynamic_batch(self, exported_onnx):
        """Batch dim should be a symbolic 'batch'; channels/H/W fixed at 1/32/32."""
        import onnx

        model = onnx.load(str(exported_onnx))
        input_proto = model.graph.input[0]
        dims = input_proto.type.tensor_type.shape.dim
        assert len(dims) == 4
        # Batch dim is symbolic
        assert dims[0].dim_param == "batch" or dims[0].dim_value == 0
        assert dims[1].dim_value == 1
        assert dims[2].dim_value == 32
        assert dims[3].dim_value == 32


class TestForwardParity:
    """Numerical parity between PyTorch shim and ONNX runtime.

    Skipped when onnxruntime isn't installed (it's only in the [quantize]
    extra). The graph-inspection invariants above already catch structural
    breakage; this test is the belt-and-braces numerical check.
    """

    def test_pytorch_onnx_outputs_match(self, tmp_path, dummy_checkpoint, exported_onnx):
        ort = pytest.importorskip("onnxruntime")
        import numpy as np

        from melody_train.export import _ExportShim
        from melody_train.model import MelodyClassifier

        # Reload the same checkpoint into a fresh model + shim
        payload = torch.load(dummy_checkpoint, weights_only=True)
        model = MelodyClassifier()
        model.load_state_dict(payload["model_state"])
        model.eval()
        shim = _ExportShim(model).eval()

        x = torch.rand(2, 1, 32, 32) * 255.0
        with torch.no_grad():
            torch_out = shim(x).numpy()

        sess = ort.InferenceSession(str(exported_onnx), providers=["CPUExecutionProvider"])
        onnx_out = sess.run(None, {"input": x.numpy()})[0]

        np.testing.assert_allclose(torch_out, onnx_out, rtol=1e-4, atol=1e-5)


# ------------------------------------------------------------------- CLI ----


class TestMainCLI:
    def test_export_main_cli_smoke(self, tmp_path, dummy_checkpoint):
        from melody_train.export import main

        out = tmp_path / "model.onnx"
        rc = main(
            [
                "--checkpoint",
                str(dummy_checkpoint),
                "--out",
                str(out),
            ]
        )
        assert rc == 0
        assert out.exists()

    def test_export_main_cli_respects_opset(self, tmp_path, dummy_checkpoint):
        import onnx

        from melody_train.export import main

        out = tmp_path / "model.onnx"
        rc = main(
            [
                "--checkpoint",
                str(dummy_checkpoint),
                "--out",
                str(out),
                "--opset",
                "17",
            ]
        )
        assert rc == 0
        model = onnx.load(str(out))
        default_opsets = [o.version for o in model.opset_import if o.domain in ("", "ai.onnx")]
        assert default_opsets[0] == 17

    def test_export_main_cli_missing_checkpoint(self, tmp_path):
        from melody_train.export import main

        rc = main(
            [
                "--checkpoint",
                str(tmp_path / "missing.pt"),
                "--out",
                str(tmp_path / "model.onnx"),
            ]
        )
        assert rc != 0
