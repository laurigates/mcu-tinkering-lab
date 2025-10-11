#!/usr/bin/env python3
"""
Verify ESP32 partition table fits within flash size constraints.
"""

# Partition definitions (offset, size in bytes)
partitions = {
    'nvs':      (0x9000,   0x6000),    # 24KB
    'otadata':  (0xF000,   0x2000),    # 8KB
    'phy_init': (0x11000,  0x1000),    # 4KB
    'ota_0':    (0x12000,  0x1CD000),  # 1.8MB
    'ota_1':    (0x1DF000, 0x1CD000),  # 1.8MB
    'spiffs':   (0x3AC000, 0x54000),   # 336KB
}

FLASH_SIZE_4MB = 0x400000  # 4MB = 4,194,304 bytes

def bytes_to_mb(bytes_val):
    """Convert bytes to MB."""
    return bytes_val / (1024 * 1024)

def bytes_to_kb(bytes_val):
    """Convert bytes to KB."""
    return bytes_val / 1024

print("=" * 60)
print("ESP32 Partition Table Verification (4MB Flash)")
print("=" * 60)

# Verify each partition
print("\nPartition Details:")
print("-" * 60)
total_size = 0
for name, (offset, size) in partitions.items():
    end = offset + size
    print(f"{name:10s} | Offset: 0x{offset:06X} | Size: 0x{size:06X} ({bytes_to_kb(size):7.1f} KB) | End: 0x{end:06X}")
    total_size += size

print("-" * 60)
print(f"{'TOTAL':10s} |                    | Size: 0x{total_size:06X} ({bytes_to_mb(total_size):7.2f} MB)")
print(f"{'FLASH':10s} |                    | Size: 0x{FLASH_SIZE_4MB:06X} ({bytes_to_mb(FLASH_SIZE_4MB):7.2f} MB)")
print("-" * 60)

# Verify gaps between partitions
print("\nPartition Continuity Check:")
print("-" * 60)
prev_name = None
prev_end = 0x9000  # Start of first partition
is_continuous = True

for name, (offset, size) in partitions.items():
    if prev_name:
        gap = offset - prev_end
        if gap > 0:
            print(f"WARNING: Gap of 0x{gap:X} bytes ({gap} bytes) between {prev_name} and {name}")
            is_continuous = False
        elif gap < 0:
            print(f"ERROR: Overlap of 0x{-gap:X} bytes ({-gap} bytes) between {prev_name} and {name}")
            is_continuous = False
        else:
            print(f"✓ {prev_name} → {name}: Continuous")

    prev_name = name
    prev_end = offset + size

if is_continuous:
    print("\n✓ All partitions are continuous (no gaps or overlaps)")

# Verify total size fits in flash
print("\nFlash Size Verification:")
print("-" * 60)
last_partition = list(partitions.items())[-1]
last_name, (last_offset, last_size) = last_partition
last_end = last_offset + last_size

if last_end <= FLASH_SIZE_4MB:
    remaining = FLASH_SIZE_4MB - last_end
    print(f"✓ Partition table fits in 4MB flash")
    print(f"  Last partition ends at: 0x{last_end:06X}")
    print(f"  Flash size:             0x{FLASH_SIZE_4MB:06X}")
    print(f"  Remaining space:        0x{remaining:06X} ({bytes_to_kb(remaining):.1f} KB)")
else:
    overflow = last_end - FLASH_SIZE_4MB
    print(f"✗ ERROR: Partition table exceeds 4MB flash by 0x{overflow:06X} ({bytes_to_kb(overflow):.1f} KB)")

# Verify 4KB alignment (ESP32 flash sector size)
print("\nAlignment Verification (4KB sectors):")
print("-" * 60)
alignment_ok = True
for name, (offset, size) in partitions.items():
    if offset % 0x1000 != 0:
        print(f"✗ {name}: Offset 0x{offset:06X} is NOT 4KB-aligned")
        alignment_ok = False
    if size % 0x1000 != 0:
        print(f"✗ {name}: Size 0x{size:06X} is NOT 4KB-aligned")
        alignment_ok = False

if alignment_ok:
    print("✓ All partitions are properly 4KB-aligned")

print("\n" + "=" * 60)
print("Verification Complete")
print("=" * 60)
