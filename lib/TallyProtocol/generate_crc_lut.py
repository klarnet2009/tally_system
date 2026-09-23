def generate_crc8_ccitt_lut():
    poly = 0x07
    lut = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        lut.append(crc)
    return lut

if __name__ == "__main__":
    lut = generate_crc8_ccitt_lut()
    print("static const uint8_t CRC8_LUT[256] = {")
    for i in range(0, 256, 8):
        chunk = ["0x{:02X}".format(x) for x in lut[i:i+8]]
        print("  " + ", ".join(chunk) + ",")
    print("};")
