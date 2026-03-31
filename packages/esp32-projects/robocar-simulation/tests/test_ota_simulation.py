"""Tests for OTA simulation module."""

import sys
from pathlib import Path

import pytest

sys.path.append(str(Path(__file__).parent.parent / "src"))

from ota_simulation import FirmwareInfo, OTASimulation, OTAState, OTAStatus


class TestOTAState:
    """Test OTA state enum."""

    def test_state_values(self):
        assert OTAState.IDLE.value == "idle"
        assert OTAState.IN_PROGRESS.value == "in_progress"
        assert OTAState.SUCCESS.value == "success"
        assert OTAState.FAILED.value == "failed"
        assert OTAState.ABORTED.value == "aborted"

    def test_all_states_exist(self):
        states = {s.value for s in OTAState}
        assert states == {"idle", "in_progress", "success", "failed", "aborted"}


class TestFirmwareInfo:
    """Test FirmwareInfo dataclass."""

    def test_construction(self):
        fw = FirmwareInfo(
            version="1.2.3",
            build_date="2024-06-01",
            size=1048576,
            checksum="abc123",
            description="Test firmware",
        )
        assert fw.version == "1.2.3"
        assert fw.size == 1048576

    def test_to_dict(self):
        fw = FirmwareInfo(
            version="1.0.0",
            build_date="2024-01-01",
            size=512000,
            checksum="deadbeef",
        )
        d = fw.to_dict()
        assert d["version"] == "1.0.0"
        assert d["size"] == 512000
        assert d["checksum"] == "deadbeef"
        assert "description" in d


class TestOTAStatus:
    """Test OTAStatus dataclass."""

    def test_idle_status(self):
        status = OTAStatus(OTAState.IDLE, 0, 0, 0)
        assert status.state == OTAState.IDLE
        assert status.progress_percent == 0

    def test_in_progress_status(self):
        status = OTAStatus(OTAState.IN_PROGRESS, 50, 500000, 1000000)
        assert status.progress_percent == 50
        assert status.bytes_downloaded == 500000

    def test_failed_status_with_error(self):
        status = OTAStatus(OTAState.FAILED, 30, 300000, 1000000, error_message="Download timeout")
        assert status.error_message == "Download timeout"

    def test_to_dict_with_firmware(self):
        fw = FirmwareInfo("1.0.0", "2024-01-01", 1000000, "abc")
        status = OTAStatus(OTAState.SUCCESS, 100, 1000000, 1000000, firmware_info=fw)
        d = status.to_dict()
        assert d["state"] == "success"
        assert d["progress_percent"] == 100
        assert "firmware_info" in d
        assert d["firmware_info"]["version"] == "1.0.0"

    def test_to_dict_without_firmware(self):
        status = OTAStatus(OTAState.IDLE, 0, 0, 0)
        d = status.to_dict()
        assert "firmware_info" not in d


class TestOTASimulation:
    """Test OTASimulation class."""

    @pytest.fixture
    def ota(self):
        config_path = str(Path(__file__).parent.parent / "config" / "robot_config.yaml")
        return OTASimulation(config_path)

    def test_initial_state(self, ota):
        assert ota.current_state == OTAState.IDLE
        assert ota.boot_partition == "factory"

    def test_partition_table_loaded(self, ota):
        assert "factory" in ota.partitions
        assert "ota_0" in ota.partitions
        assert "ota_1" in ota.partitions
        assert "nvs" in ota.partitions

    def test_factory_firmware_exists(self, ota):
        assert "factory" in ota.firmware_versions
        assert ota.firmware_versions["factory"].version == "1.0.0"

    def test_get_next_ota_partition_from_factory(self, ota):
        ota.boot_partition = "factory"
        assert ota._get_next_ota_partition() == "ota_0"

    def test_get_next_ota_partition_alternates(self, ota):
        ota.boot_partition = "ota_0"
        assert ota._get_next_ota_partition() == "ota_1"

        ota.boot_partition = "ota_1"
        assert ota._get_next_ota_partition() == "ota_0"

    def test_not_ota_ready_without_wifi(self, ota):
        """OTA should not be ready without a WiFi manager."""
        assert not ota.is_ota_ready()

    def test_start_update_fails_without_wifi(self, ota):
        result = ota.start_update("https://example.com/firmware.bin", "2.0.0")
        assert result is False
        assert ota.current_status.error_message == "System not ready for OTA"

    def test_set_boot_partition_valid(self, ota):
        assert ota.set_boot_partition("ota_0") is True
        assert ota.boot_partition == "ota_0"

    def test_set_boot_partition_invalid_name(self, ota):
        assert ota.set_boot_partition("nonexistent") is False

    def test_set_boot_partition_invalid_ota1(self, ota):
        """ota_1 is not valid initially, so setting boot to it should fail."""
        assert ota.set_boot_partition("ota_1") is False

    def test_rollback_to_factory(self, ota):
        ota.boot_partition = "ota_0"
        assert ota.rollback_to_factory() is True
        assert ota.boot_partition == "factory"

    def test_rollback_blocked_during_update(self, ota):
        ota.current_state = OTAState.IN_PROGRESS
        assert ota.rollback_to_factory() is False

    def test_abort_when_not_in_progress(self, ota):
        assert ota.abort_update() is False

    def test_get_system_info(self, ota):
        info = ota.get_system_info()
        assert "current_state" in info
        assert "boot_partition" in info
        assert "partitions" in info
        assert "firmware_versions" in info
        assert info["current_state"] == "idle"

    def test_get_partition_info(self, ota):
        info = ota.get_partition_info()
        assert "factory" in info
        assert info["factory"]["type"] == "app"
        assert info["factory"]["subtype"] == "factory"
