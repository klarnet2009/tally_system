#!/usr/bin/env python3
def generate_crc8_ccitt_lut():
    lut = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        lut.append(crc)
    return lut

lut = generate_crc8_ccitt_lut()
print("static const uint8_t CRC8_LUT[256] = {")
for i in range(0, 256, 8):
    row = ", ".join(f"0x{v:02X}" for v in lut[i:i+8])
    if i < 256 - 8:
        row += ","
    print(f"  {row}")
print("};")
