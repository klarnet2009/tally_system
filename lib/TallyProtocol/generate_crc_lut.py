def generate_crc8_lut(poly=0x07):
    lut = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        lut.append(crc)

    out = "static const uint8_t CRC8_LUT[256] = {\n"
    for i in range(0, 256, 8):
        out += "    " + ", ".join([f"0x{lut[i+j]:02X}" for j in range(8)]) + ",\n"
    out += "};"
    return out

print(generate_crc8_lut())
