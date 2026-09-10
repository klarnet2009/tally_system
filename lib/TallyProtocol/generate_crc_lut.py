#!/usr/bin/env python3
def generate_crc8_lut(poly=0x07):
    lut = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFF
        lut.append(crc)
    return lut

lut = generate_crc8_lut()
print("static const uint8_t CRC8_LUT[256] = {")
for i in range(0, 256, 8):
    print("  " + ", ".join(f"0x{x:02X}" for x in lut[i:i+8]) + ",")
print("};")
