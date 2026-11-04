"""Tests for the bridge's serial message framing (type byte + data + CRC8)."""

import sys
from pathlib import Path

import pytest

sys.path.append(str(Path(__file__).parent.parent / "src"))

from communication_bridge import I2CMessage, MessageType


class TestMessageType:
    """MessageType values must be unique so a type byte decodes unambiguously."""

    def test_no_duplicate_values(self):
        """All message types must have unique values."""
        values = [m.value for m in MessageType]
        assert len(values) == len(set(values))


class TestI2CMessage:
    """Test message construction and serialization."""

    def test_construction_calculates_checksum(self):
        msg = I2CMessage(MessageType.MOVE, b"\x01\x80")
        assert msg.checksum != 0

    def test_to_bytes_format(self):
        """to_bytes should produce: type_byte + data + checksum_byte"""
        msg = I2CMessage(MessageType.MOVE, b"\x01\x80")
        raw = msg.to_bytes()
        assert raw[0] == MessageType.MOVE.value
        assert raw[1:-1] == b"\x01\x80"
        assert raw[-1] == msg.checksum

    def test_from_bytes_roundtrip(self):
        """from_bytes(to_bytes()) should reconstruct the original message."""
        original = I2CMessage(MessageType.SERVO, b"\x02\x5a")
        raw = original.to_bytes()
        restored = I2CMessage.from_bytes(raw)
        assert restored.message_type == original.message_type
        assert restored.data == original.data
        assert restored.checksum == original.checksum

    def test_from_bytes_too_short(self):
        with pytest.raises(ValueError, match="too short"):
            I2CMessage.from_bytes(b"\x01")

    def test_empty_data(self):
        msg = I2CMessage(MessageType.ENTER_MAINTENANCE_MODE, b"")
        raw = msg.to_bytes()
        assert len(raw) == 2  # type + checksum
        restored = I2CMessage.from_bytes(raw)
        assert restored.data == b""

    def test_checksum_detects_corruption(self):
        msg = I2CMessage(MessageType.MOVE, b"\x01\x80")
        raw = bytearray(msg.to_bytes())
        raw[1] ^= 0xFF  # corrupt data byte
        restored = I2CMessage.from_bytes(bytes(raw))
        expected_checksum = restored._calculate_checksum()
        assert restored.checksum != expected_checksum

    def test_ota_message_types(self):
        """All OTA message types should be constructable."""
        for mt in [
            MessageType.ENTER_MAINTENANCE_MODE,
            MessageType.BEGIN_OTA,
            MessageType.GET_OTA_STATUS,
            MessageType.GET_VERSION,
            MessageType.REBOOT,
        ]:
            msg = I2CMessage(mt, b"\x00")
            assert msg.message_type == mt
            raw = msg.to_bytes()
            assert len(raw) >= 2
