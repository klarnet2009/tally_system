def crc8_ccitt():
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

lut = crc8_ccitt()
print("static const uint8_t crc8_lut[256] = {")
for i in range(0, 256, 16):
    print("  " + ", ".join(f"0x{x:02X}" for x in lut[i:i+16]) + ",")
print("};")
