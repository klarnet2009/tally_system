#!/usr/bin/env python3
print("const uint8_t CRC8_LUT[256] = {")
poly = 0x07
lines = []
for i in range(0, 256, 8):
    chunk = []
    for j in range(8):
        crc = i + j
        for b in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        chunk.append(f"0x{crc:02x}")
    lines.append("    " + ", ".join(chunk) + ",")
print("\n".join(lines))
print("};")
