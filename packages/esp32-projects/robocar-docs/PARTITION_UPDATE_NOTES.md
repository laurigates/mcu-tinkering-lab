# OTA Partition Table Update - Phase 1.2

## Overview
Updated partition tables for both robocar-main (Heltec WiFi LoRa 32) and robocar-camera (ESP32-CAM) to properly support OTA updates within 4MB flash constraints.

## Changes Made

### Previous Partition Layout (6MB - INCORRECT)
The previous partition tables were designed for 6MB flash, which exceeded the actual 4MB flash size on both boards:
- nvs: 24KB
- phy_init: 4KB
- factory: 1.5MB
- ota_0: 1.5MB
- ota_1: 1.5MB
- ota_data: 8KB
- storage: 1.4MB
- **Total: ~6MB** ❌

### New Partition Layout (4MB - CORRECT)
```
Partition   | Offset    | Size      | Description
----------- | --------- | --------- | -----------
nvs         | 0x009000  | 24KB      | Non-Volatile Storage
otadata     | 0x00F000  | 8KB       | OTA selection data
phy_init    | 0x011000  | 4KB       | PHY init data
ota_0       | 0x012000  | 1.8MB     | OTA app partition 0
ota_1       | 0x1DF000  | 1.8MB     | OTA app partition 1
spiffs      | 0x3AC000  | 336KB     | File storage for AI models
----------- | --------- | --------- | -----------
TOTAL       |           | 3.96MB    | Fits in 4MB flash ✓
```

### Key Changes
1. **Removed factory partition**: Not needed with dual OTA partitions
2. **Increased OTA partition sizes**: From 1.5MB to 1.8MB each
3. **Reduced SPIFFS size**: From 1.4MB to 336KB (sufficient for AI models)
4. **Reorganized layout**: Moved otadata earlier for better reliability
5. **Total size**: Fits exactly within 4MB flash

## Board Specifications

### Heltec WiFi LoRa 32 (robocar-main)
- **MCU**: ESP32
- **Flash**: 4MB
- **Features**: WiFi, LoRa, OLED display

### ESP32-CAM (robocar-camera)
- **MCU**: ESP32
- **Flash**: 4MB
- **Features**: WiFi, Camera (OV2640)

## Verification

Run the verification script to validate the partition table:
```bash
python3 verify_partitions.py
```

This confirms:
- ✓ All partitions fit in 4MB flash
- ✓ No gaps or overlaps
- ✓ Proper 4KB alignment
- ✓ Continuous memory layout

## Build Instructions

### Clean Build (Recommended)
When changing partition tables, it's recommended to perform a clean build and erase flash:

```bash
# For robocar-main
cd packages/esp32-projects/robocar-main
idf.py fullclean
idf.py erase-flash
idf.py build flash monitor

# For robocar-camera
cd packages/esp32-projects/robocar-camera
idf.py fullclean
idf.py erase-flash
idf.py build flash monitor
```

### Verify Partition Table
After flashing, verify the partition table was applied correctly:
```bash
idf.py partition-table
```

## OTA Update Workflow

With the new partition table, OTA updates will work as follows:

1. **Initial Flash**: App is written to `ota_0` partition
2. **First OTA Update**: New firmware downloads to `ota_1` partition
3. **Boot**: Device boots from `ota_1` (new firmware)
4. **Second OTA Update**: New firmware downloads to `ota_0` partition
5. **Rollback**: If boot fails, device automatically reverts to previous partition

## Testing Checklist

- [ ] Build robocar-main with new partition table
- [ ] Flash and verify robocar-main boots successfully
- [ ] Verify WiFi and LoRa functionality on robocar-main
- [ ] Build robocar-camera with new partition table
- [ ] Flash and verify robocar-camera boots successfully
- [ ] Verify WiFi and camera functionality on robocar-camera
- [ ] Test OTA update on both boards
- [ ] Verify SPIFFS functionality (AI model storage)
- [ ] Test OTA rollback functionality

## Troubleshooting

### Error: "Partition table does not fit in configured flash size"
- Verify flash size is set to 4MB in `idf.py menuconfig`
- Navigate to: `Serial flasher config` → `Flash size` → Select `4 MB`

### Error: "ota_begin error err=0x104"
- This indicates insufficient space for OTA
- Verify partition table is correctly flashed
- Run `idf.py erase-flash` and reflash

### Error: "Partition table MD5 checksum failed"
- The partition table has been modified
- Run `idf.py erase-flash` to clear old partition table
- Reflash with new partition table

## Related Issues
- Issue #22: OTA Phase 1.2 - Modify Partition Tables for OTA
- Issue #14: Parent OTA implementation issue

## References
- [ESP-IDF Partition Tables Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html)
- [ESP-IDF OTA Updates](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html)
