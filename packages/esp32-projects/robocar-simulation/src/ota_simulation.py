"""
OTA (Over-The-Air) Update Simulation Module

This module simulates the ESP32 OTA functionality including:
- Partition management (factory, ota_0, ota_1)
- Firmware download simulation
- Update verification and switching
- Boot partition management
- OTA status reporting

Based on ESP-IDF OTA system and partition table structure.
"""

import hashlib
import random
import threading
import time
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path

import yaml

from error_handling import ErrorSeverity, get_error_handler


class OTAState(Enum):
    """OTA operation states matching ESP-IDF implementation"""

    IDLE = "idle"
    IN_PROGRESS = "in_progress"
    SUCCESS = "success"
    FAILED = "failed"
    ABORTED = "aborted"


class PartitionType(Enum):
    """ESP32 partition types from partition table"""

    APP_FACTORY = "factory"
    APP_OTA_0 = "ota_0"
    APP_OTA_1 = "ota_1"
    DATA_NVS = "nvs"
    DATA_PHY = "phy"
    DATA_OTA = "ota"
    DATA_SPIFFS = "spiffs"


@dataclass
class PartitionInfo:
    """ESP32 partition information"""

    name: str
    type: str
    subtype: str
    offset: int
    size: int
    flags: str = ""

    def to_hex_offset(self) -> str:
        """Convert offset to hex string"""
        return f"0x{self.offset:x}"

    def to_hex_size(self) -> str:
        """Convert size to hex string"""
        return f"0x{self.size:x}"


@dataclass
class FirmwareInfo:
    """Firmware image information"""

    version: str
    build_date: str
    size: int
    checksum: str
    description: str = ""

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization"""
        return asdict(self)


@dataclass
class OTAStatus:
    """OTA operation status"""

    state: OTAState
    progress_percent: int
    bytes_downloaded: int
    total_bytes: int
    error_message: str = ""
    firmware_info: FirmwareInfo | None = None

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization"""
        result = {
            "state": self.state.value,
            "progress_percent": self.progress_percent,
            "bytes_downloaded": self.bytes_downloaded,
            "total_bytes": self.total_bytes,
            "error_message": self.error_message,
        }
        if self.firmware_info:
            result["firmware_info"] = self.firmware_info.to_dict()
        return result


class OTASimulation:
    """
    Simulates ESP32 OTA functionality for robot simulation

    This implementation follows ESP-IDF OTA patterns including:
    - Dual partition system (ota_0, ota_1)
    - Boot partition management
    - Firmware verification
    - Progress tracking
    """

    def __init__(self, config_path: str):
        # Initialize error handling
        self.error_handler = get_error_handler()
        self.error_handler.register_component("ota_simulation")
        self._register_recovery_strategies()

        self._status_lock = threading.Lock()
        self.config = self._load_config(config_path)

        # Load partition table (matching ESP32 partition structure)
        self.partitions = self._load_partition_table()

        # OTA state
        self.current_state = OTAState.IDLE
        self.current_status = OTAStatus(OTAState.IDLE, 0, 0, 0)

        # Partition management
        self.boot_partition = "factory"  # Currently running partition
        self.ota_partition = None  # Next OTA partition to use
        self.ota_data = self._load_ota_data()

        # Download simulation
        self.download_thread: threading.Thread | None = None
        self.abort_download = False

        # Firmware versions (simulated)
        self.firmware_versions = self._initialize_firmware_versions()

        # WiFi dependency
        self.wifi_manager = None  # Will be set by robot system

    def _register_recovery_strategies(self):
        """Register recovery strategies for OTA simulation errors"""

        def ota_recovery(error) -> bool:
            """Recovery strategy for OTA-related errors"""
            try:
                # Abort any in-progress operation
                if hasattr(self, "current_state") and self.current_state == OTAState.IN_PROGRESS:
                    self.abort_update()

                # Reset to idle state
                if hasattr(self, "current_state"):
                    self.current_state = OTAState.IDLE
                    self.current_status = OTAStatus(OTAState.IDLE, 0, 0, 0)

                return True
            except Exception:
                return False

        def partition_recovery(error) -> bool:
            """Recovery strategy for partition-related errors"""
            try:
                # Ensure boot partition is valid
                if hasattr(self, "boot_partition"):
                    if self.boot_partition not in ["factory", "ota_0", "ota_1"]:
                        self.boot_partition = "factory"

                # Reset OTA data to safe state
                if hasattr(self, "ota_data"):
                    self.ota_data = {
                        "seq": 0,
                        "ota_0": {"valid": True, "seq": 0},
                        "ota_1": {"valid": False, "seq": 0},
                    }

                return True
            except Exception:
                return False

        # Register strategies
        if hasattr(self, "error_handler"):
            self.error_handler.register_recovery_strategy("ota_simulation", ota_recovery)
            self.error_handler.register_recovery_strategy("ota_simulation", partition_recovery)

    def _load_config(self, config_path: str) -> dict:
        """Load configuration from YAML file"""
        config_file = Path(config_path)
        if not config_file.exists():
            config_file = Path(__file__).parent.parent / "config" / "robot_config.yaml"

        with open(config_file) as f:
            return yaml.safe_load(f)

    def _load_partition_table(self) -> dict[str, PartitionInfo]:
        """Load ESP32 partition table (matching actual ESP32 layout)"""
        # Based on the actual partitions.csv from the ESP32 project
        partitions = {
            "nvs": PartitionInfo("nvs", "data", "nvs", 0x9000, 0x6000),
            "phy_init": PartitionInfo("phy_init", "data", "phy", 0xF000, 0x1000),
            "factory": PartitionInfo("factory", "app", "factory", 0x10000, 0x180000),
            "ota_0": PartitionInfo("ota_0", "app", "ota_0", 0x190000, 0x180000),
            "ota_1": PartitionInfo("ota_1", "app", "ota_1", 0x310000, 0x180000),
            "ota_data": PartitionInfo("ota_data", "data", "ota", 0x490000, 0x2000),
            "storage": PartitionInfo("storage", "data", "spiffs", 0x492000, 0x16E000),
        }

        return partitions

    def _load_ota_data(self) -> dict:
        """Load OTA data partition information"""
        # Simulate OTA data partition content
        # In real ESP32, this tracks which partition is active
        return {
            "seq": 0,  # Sequence number for tracking updates
            "ota_0": {"valid": True, "seq": 0},
            "ota_1": {"valid": False, "seq": 0},
        }

    def _initialize_firmware_versions(self) -> dict[str, FirmwareInfo]:
        """Initialize simulated firmware versions"""
        versions = {}

        # Current firmware (factory)
        versions["factory"] = FirmwareInfo(
            version="1.0.0",
            build_date="2024-01-15",
            size=1024 * 1024,  # 1MB
            checksum="a1b2c3d4e5f6",
            description="Factory firmware",
        )

        # OTA partition 0 (if valid)
        if self.ota_data["ota_0"]["valid"]:
            versions["ota_0"] = FirmwareInfo(
                version="1.1.0",
                build_date="2024-02-01",
                size=1024 * 1024,
                checksum="b2c3d4e5f6a1",
                description="OTA update v1.1.0",
            )

        return versions

    def _save_ota_data(self):
        """Simulate writing OTA data to partition"""
        try:
            # In real ESP32, this would write to flash
            # Here we just update our internal state
            print("OTA simulation: Updated OTA data partition")
            self.error_handler.report_component_success("ota_simulation")
        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "ota_data_save_failed",
                f"Failed to save OTA data: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )

    def set_wifi_manager(self, wifi_manager):
        """Set WiFi manager dependency"""
        self.wifi_manager = wifi_manager

    def is_ota_ready(self) -> bool:
        """Check if system is ready for OTA operations"""
        try:
            # Check WiFi connectivity
            if not self.wifi_manager or not self.wifi_manager.is_ota_ready():
                return False

            # Check if already in progress
            if self.current_state == OTAState.IN_PROGRESS:
                return False

            # Check partition availability
            if not self._get_next_ota_partition():
                return False

            self.error_handler.report_component_success("ota_simulation")
            return True

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "ota_ready_check_failed",
                f"OTA ready check failed: {str(e)}",
                e,
                ErrorSeverity.LOW,
            )
            return False

    def _get_next_ota_partition(self) -> str | None:
        """Get the next OTA partition to use"""
        # Determine which OTA partition to use based on current boot partition
        if self.boot_partition == "factory" or self.boot_partition == "ota_1":
            return "ota_0"
        elif self.boot_partition == "ota_0":
            return "ota_1"
        else:
            return "ota_0"  # Default fallback

    def start_update(self, firmware_url: str, version: str = None) -> bool:
        """Start OTA update process"""
        try:
            if not self.is_ota_ready():
                self.current_status.error_message = "System not ready for OTA"
                return False

            # Determine target partition
            self.ota_partition = self._get_next_ota_partition()
            if not self.ota_partition:
                self.current_status.error_message = "No available OTA partition"
                return False

            # Create firmware info for download
            firmware_info = FirmwareInfo(
                version=version or f"update-{int(time.time())}",
                build_date=time.strftime("%Y-%m-%d"),
                size=random.randint(800000, 1200000),  # Random size 800KB-1.2MB
                checksum="",
                description=f"OTA update from {firmware_url}",
            )

            # Initialize status
            self.current_state = OTAState.IN_PROGRESS
            self.current_status = OTAStatus(
                state=OTAState.IN_PROGRESS,
                progress_percent=0,
                bytes_downloaded=0,
                total_bytes=firmware_info.size,
                firmware_info=firmware_info,
            )

            self.abort_download = False

            # Start download simulation
            self.download_thread = threading.Thread(
                target=self._simulate_download, args=(firmware_url, firmware_info), daemon=True
            )
            self.download_thread.start()

            print(f"OTA simulation: Started update to partition {self.ota_partition}")
            print(f"  Firmware URL: {firmware_url}")
            print(f"  Version: {firmware_info.version}")
            print(f"  Size: {firmware_info.size} bytes")

            self.error_handler.report_component_success("ota_simulation")
            return True

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "ota_start_failed",
                f"Failed to start OTA update: {str(e)}",
                e,
                ErrorSeverity.HIGH,
            )
            self.current_state = OTAState.FAILED
            self.current_status.state = OTAState.FAILED
            self.current_status.error_message = str(e)
            return False

    def _simulate_download(self, firmware_url: str, firmware_info: FirmwareInfo):
        """Simulate firmware download and installation"""
        try:
            total_bytes = firmware_info.size
            download_speed = 50000  # 50KB/s (realistic for WiFi)
            chunk_size = 4096  # 4KB chunks

            bytes_downloaded = 0
            hasher = hashlib.md5()

            print(f"OTA simulation: Starting download from {firmware_url}")

            while bytes_downloaded < total_bytes and not self.abort_download:
                # Simulate network chunk download
                chunk_bytes = min(chunk_size, total_bytes - bytes_downloaded)
                time.sleep(chunk_bytes / download_speed)  # Simulate download time

                # Simulate data processing
                chunk_data = b"X" * chunk_bytes  # Dummy data
                hasher.update(chunk_data)
                bytes_downloaded += chunk_bytes

                # Update progress (lock protects shared status fields)
                progress = int((bytes_downloaded / total_bytes) * 100)
                with self._status_lock:
                    self.current_status.bytes_downloaded = bytes_downloaded
                    self.current_status.progress_percent = progress

                # Periodic progress updates
                if progress % 10 == 0 and progress > 0:
                    print(
                        f"OTA simulation: Download progress {progress}% "
                        f"({bytes_downloaded}/{total_bytes} bytes)"
                    )

                # Simulate occasional WiFi issues
                if random.random() < 0.02:  # 2% chance of delay
                    print("OTA simulation: WiFi congestion, slowing download...")
                    time.sleep(0.5)

            if self.abort_download:
                self._handle_download_abort()
                return

            # Download complete - simulate verification
            print("OTA simulation: Download complete, verifying firmware...")
            firmware_info.checksum = hasher.hexdigest()

            time.sleep(1.0)  # Simulate verification time

            # Simulate verification (with small chance of failure)
            if random.random() < 0.05:  # 5% chance of verification failure
                self._handle_verification_failure("Checksum verification failed")
                return

            # Success - update OTA data
            self._handle_download_success(firmware_info)

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "download_simulation_failed",
                f"Download simulation failed: {str(e)}",
                e,
                ErrorSeverity.HIGH,
            )
            self._handle_download_failure(str(e))

    def _handle_download_success(self, firmware_info: FirmwareInfo):
        """Handle successful firmware download and installation"""
        try:
            print("OTA simulation: Firmware verification successful")

            # Update OTA data partition
            self.ota_data["seq"] += 1
            self.ota_data[self.ota_partition] = {"valid": True, "seq": self.ota_data["seq"]}

            # Store firmware info
            self.firmware_versions[self.ota_partition] = firmware_info

            # Update status
            self.current_state = OTAState.SUCCESS
            self.current_status.state = OTAState.SUCCESS
            self.current_status.progress_percent = 100

            self._save_ota_data()

            print("OTA simulation: Update successful!")
            print(f"  Partition: {self.ota_partition}")
            print(f"  Version: {firmware_info.version}")
            print(f"  Checksum: {firmware_info.checksum}")
            print("  Ready to boot from new partition on next restart")

            self.error_handler.report_component_success("ota_simulation")

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "ota_success_handling_failed",
                f"Failed to handle successful OTA: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )
            self._handle_download_failure(f"Post-download processing failed: {str(e)}")

    def _handle_download_failure(self, error_message: str):
        """Handle failed firmware download"""
        self.current_state = OTAState.FAILED
        self.current_status.state = OTAState.FAILED
        self.current_status.error_message = error_message

        print(f"OTA simulation: Update failed - {error_message}")

    def _handle_download_abort(self):
        """Handle aborted firmware download"""
        self.current_state = OTAState.ABORTED
        self.current_status.state = OTAState.ABORTED
        self.current_status.error_message = "Update aborted by user"

        print("OTA simulation: Update aborted")

    def _handle_verification_failure(self, error_message: str):
        """Handle firmware verification failure"""
        self.current_state = OTAState.FAILED
        self.current_status.state = OTAState.FAILED
        self.current_status.error_message = error_message

        print(f"OTA simulation: Verification failed - {error_message}")

    def abort_update(self) -> bool:
        """Abort ongoing OTA update"""
        try:
            if self.current_state != OTAState.IN_PROGRESS:
                return False

            print("OTA simulation: Aborting update...")
            self.abort_download = True

            # Wait for download thread to finish
            if self.download_thread and self.download_thread.is_alive():
                self.download_thread.join(timeout=5.0)

            self.error_handler.report_component_success("ota_simulation")
            return True

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "ota_abort_failed",
                f"Failed to abort OTA update: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )
            return False

    def get_status(self) -> OTAStatus:
        """Get current OTA status"""
        return self.current_status

    def get_status_dict(self) -> dict:
        """Get current OTA status as dictionary"""
        return self.current_status.to_dict()

    def set_boot_partition(self, partition_name: str) -> bool:
        """Set the boot partition for next restart"""
        try:
            if partition_name not in ["factory", "ota_0", "ota_1"]:
                return False

            # Check if partition is valid
            if partition_name in ["ota_0", "ota_1"]:
                if not self.ota_data[partition_name]["valid"]:
                    return False

            # Update boot partition
            old_partition = self.boot_partition
            self.boot_partition = partition_name

            print(
                f"OTA simulation: Boot partition changed from {old_partition} to {partition_name}"
            )
            print("  Will take effect on next restart")

            self.error_handler.report_component_success("ota_simulation")
            return True

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "boot_partition_change_failed",
                f"Failed to set boot partition: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )
            return False

    def get_boot_partition(self) -> str:
        """Get current boot partition"""
        return self.boot_partition

    def get_partition_info(self) -> dict[str, dict]:
        """Get partition table information"""
        result = {}
        for name, partition in self.partitions.items():
            result[name] = {
                "type": partition.type,
                "subtype": partition.subtype,
                "offset": partition.to_hex_offset(),
                "size": partition.to_hex_size(),
                "flags": partition.flags,
            }
        return result

    def get_firmware_versions(self) -> dict[str, dict]:
        """Get firmware version information for all partitions"""
        result = {}
        for partition, firmware in self.firmware_versions.items():
            result[partition] = firmware.to_dict()
        return result

    def rollback_to_factory(self) -> bool:
        """Rollback to factory firmware"""
        try:
            if self.current_state == OTAState.IN_PROGRESS:
                return False  # Cannot rollback during update

            # Set boot partition to factory
            if self.set_boot_partition("factory"):
                print("OTA simulation: Rollback to factory firmware initiated")
                print("  Will boot from factory partition on next restart")
                return True

            return False

        except Exception as e:
            self.error_handler.handle_error(
                "ota_simulation",
                "factory_rollback_failed",
                f"Factory rollback failed: {str(e)}",
                e,
                ErrorSeverity.MEDIUM,
            )
            return False

    def get_system_info(self) -> dict:
        """Get comprehensive OTA system information"""
        return {
            "current_state": self.current_state.value,
            "boot_partition": self.boot_partition,
            "ota_partition": self.ota_partition,
            "partitions": self.get_partition_info(),
            "firmware_versions": self.get_firmware_versions(),
            "ota_data": self.ota_data.copy(),
            "ota_ready": self.is_ota_ready(),
            "current_status": self.get_status_dict(),
        }


def main():
    """Test OTA simulation"""
    config_path = "../config/robot_config.yaml"

    # Create OTA simulation
    ota = OTASimulation(config_path)

    print("OTA Simulation Test")
    print("=" * 40)

    try:
        # Show system info
        info = ota.get_system_info()
        print("System Information:")
        print(f"  Boot partition: {info['boot_partition']}")
        print(f"  OTA ready: {info['ota_ready']}")

        print("\nPartition Table:")
        for name, partition in info["partitions"].items():
            print(
                f"  {name}: {partition['type']}/{partition['subtype']} "
                f"at {partition['offset']} ({partition['size']})"
            )

        print("\nFirmware Versions:")
        for partition, firmware in info["firmware_versions"].items():
            print(f"  {partition}: {firmware['version']} ({firmware['build_date']})")

        # Simulate OTA update (without actual WiFi)
        print("\nStarting OTA update simulation...")
        if ota.start_update("https://example.com/firmware.bin", "1.2.0"):
            # Monitor progress
            while ota.get_status().state == OTAState.IN_PROGRESS:
                status = ota.get_status()
                print(
                    f"Progress: {status.progress_percent}% "
                    f"({status.bytes_downloaded}/{status.total_bytes} bytes)"
                )
                time.sleep(2)

            # Final status
            final_status = ota.get_status()
            print(f"\nOTA Complete: {final_status.state.value}")
            if final_status.error_message:
                print(f"Error: {final_status.error_message}")

            # Show updated system info
            info = ota.get_system_info()
            print("\nUpdated firmware versions:")
            for partition, firmware in info["firmware_versions"].items():
                print(f"  {partition}: {firmware['version']} ({firmware['build_date']})")

    except KeyboardInterrupt:
        print("\nAborting OTA simulation...")
        ota.abort_update()

    print("\nOTA simulation test complete")


if __name__ == "__main__":
    main()
