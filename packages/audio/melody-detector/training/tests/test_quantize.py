"""Tests for the melody-detector ESP-PPQ quantization pipeline.

The lightweight tests (dataclass shape, ``ImportError`` surface when the
quantize extra is missing) always run. The end-to-end smoke test
(``gen → train → export → quantize → assert .espdl``) is the slowest
test in the suite and is gated by ``pytest.importorskip("esp_ppq")``,
so it only runs when ESP-PPQ is installed (i.e. under
``--extra quantize`` / ``just sync-quantize``).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))


# ----------------------------------------------------- QuantizeConfig shape ----


class TestQuantizeConfig:
    def test_instantiate_with_required_fields(self, tmp_path):
        from melody_train.quantize import QuantizeConfig

        cfg = QuantizeConfig(
            onnx_path=tmp_path / "model.onnx",
            data_dir=tmp_path / "data",
            out_path=tmp_path / "model.espdl",
        )
        assert cfg.onnx_path == tmp_path / "model.onnx"
        assert cfg.data_dir == tmp_path / "data"
        assert cfg.out_path == tmp_path / "model.espdl"

    def test_defaults(self, tmp_path):
        from melody_train.quantize import QuantizeConfig

        cfg = QuantizeConfig(
            onnx_path=tmp_path / "model.onnx",
            data_dir=tmp_path / "data",
            out_path=tmp_path / "model.espdl",
        )
        assert cfg.calib_steps == 32
        assert cfg.batch_size == 16
        assert cfg.target == "esp32s3"
        assert cfg.num_of_bits == 8
        assert cfg.val_fraction == 0.1
        assert cfg.seed == 0
        assert cfg.device == "cpu"

    def test_is_frozen(self, tmp_path):
        from melody_train.quantize import QuantizeConfig

        cfg = QuantizeConfig(
            onnx_path=tmp_path / "model.onnx",
            data_dir=tmp_path / "data",
            out_path=tmp_path / "model.espdl",
        )
        with pytest.raises((AttributeError, TypeError)):
            cfg.calib_steps = 64  # type: ignore[misc]


class TestQuantizeResult:
    def test_instantiate(self, tmp_path):
        from melody_train.quantize import QuantizeResult

        result = QuantizeResult(
            espdl_path=tmp_path / "model.espdl",
            info_path=tmp_path / "model.info",
            json_path=tmp_path / "model.json",
            target="esp32s3",
            num_of_bits=8,
        )
        assert result.target == "esp32s3"
        assert result.num_of_bits == 8

    def test_is_frozen(self, tmp_path):
        from melody_train.quantize import QuantizeResult

        result = QuantizeResult(
            espdl_path=tmp_path / "model.espdl",
            info_path=tmp_path / "model.info",
            json_path=tmp_path / "model.json",
            target="esp32s3",
            num_of_bits=8,
        )
        with pytest.raises((AttributeError, TypeError)):
            result.target = "esp32"  # type: ignore[misc]


# --------------------------------------------------- behaviour without esp-ppq


class TestQuantizeWithoutEspPpq:
    """Surface a clear error if the user runs `quantize()` without esp-ppq."""

    def test_raises_filenotfound_when_onnx_missing(self, tmp_path):
        from melody_train.quantize import QuantizeConfig, quantize

        cfg = QuantizeConfig(
            onnx_path=tmp_path / "missing.onnx",
            data_dir=tmp_path / "data",
            out_path=tmp_path / "model.espdl",
        )
        with pytest.raises(FileNotFoundError):
            quantize(cfg)


# --------------------------------------------------------- end-to-end smoke


@pytest.mark.slow
class TestEndToEnd:
    """Gen → train → export → quantize, asserting the final .espdl drops out.

    Skipped unless esp-ppq is installed. Kept small (32 samples/class,
    2 epochs, 2 calib steps) — this is still the slowest test in the
    suite when it does run.
    """

    def test_full_pipeline(self, tmp_path):
        pytest.importorskip("esp_ppq")

        from melody_train.export import ExportConfig, export
        from melody_train.gen import generate_dataset
        from melody_train.quantize import QuantizeConfig, quantize
        from melody_train.train import TrainConfig, train

        data_dir = tmp_path / "data"
        out_dir = tmp_path / "runs"
        out_dir.mkdir()

        generate_dataset(seed=0, out_dir=data_dir, per_class=32, image_size=32)

        train_cfg = TrainConfig(
            data_dir=data_dir,
            out_dir=out_dir,
            epochs=2,
            batch_size=16,
            seed=0,
        )
        train_result = train(train_cfg)
        assert train_result.checkpoint_path.exists()

        onnx_path = out_dir / "model.onnx"
        export(ExportConfig(checkpoint_path=train_result.checkpoint_path, out_path=onnx_path))
        assert onnx_path.exists()

        espdl_path = out_dir / "model.espdl"
        result = quantize(
            QuantizeConfig(
                onnx_path=onnx_path,
                data_dir=data_dir,
                out_path=espdl_path,
                calib_steps=2,
                batch_size=8,
                seed=0,
            )
        )

        assert result.espdl_path.exists()
        assert result.espdl_path.stat().st_size > 0
        # MelodyClassifier is ~3 KB INT8; .espdl wrapper adds metadata but
        # should still be well under 50 KB. Surfaces gross regressions
        # (e.g. weights stored as float32, debug data) early.
        assert result.espdl_path.stat().st_size < 50_000

    def test_info_and_json_sidecars(self, tmp_path):
        pytest.importorskip("esp_ppq")

        from melody_train.export import ExportConfig, export
        from melody_train.gen import generate_dataset
        from melody_train.quantize import QuantizeConfig, quantize
        from melody_train.train import TrainConfig, train

        data_dir = tmp_path / "data"
        out_dir = tmp_path / "runs"
        out_dir.mkdir()

        generate_dataset(seed=0, out_dir=data_dir, per_class=32, image_size=32)

        train_result = train(
            TrainConfig(
                data_dir=data_dir,
                out_dir=out_dir,
                epochs=2,
                batch_size=16,
                seed=0,
            )
        )

        onnx_path = out_dir / "model.onnx"
        export(ExportConfig(checkpoint_path=train_result.checkpoint_path, out_path=onnx_path))

        result = quantize(
            QuantizeConfig(
                onnx_path=onnx_path,
                data_dir=data_dir,
                out_path=out_dir / "model.espdl",
                calib_steps=2,
                batch_size=8,
                seed=0,
            )
        )

        # .json sidecar is structured quantization metadata — must parse.
        assert result.json_path.exists(), f"expected JSON metadata at {result.json_path}"
        json.loads(result.json_path.read_text())

        # .info sidecar is a human-readable layer dump — non-empty and
        # contains some recognisable header/structure.
        assert result.info_path.exists(), f"expected info dump at {result.info_path}"
        info_text = result.info_path.read_text()
        assert len(info_text) > 0
        # Loose check: the info file should at least mention layer / op
        # / quant terminology somewhere. Avoid over-specifying ESP-PPQ's
        # exact format, which may vary across releases.
        info_lower = info_text.lower()
        assert any(kw in info_lower for kw in ("layer", "op", "quant", "scale")), (
            f"info file doesn't look like a layer dump:\n{info_text[:500]}"
        )
