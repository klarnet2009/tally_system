#!/usr/bin/env python3
print("static const uint8_t CRC8_LUT[256] = {")
for i in range(256):
    crc = i
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    print(f"  0x{crc:02x},", end="")
    if i % 8 == 7:
        print()
print("};")
