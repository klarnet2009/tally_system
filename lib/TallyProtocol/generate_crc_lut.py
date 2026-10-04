def generate_lut():
    lut = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        lut.append(crc)

    print("static const uint8_t CRC8_LUT[256] = {")
    for i in range(0, 256, 12):
        chunk = lut[i:i+12]
        print("    " + ", ".join(f"0x{x:02X}" for x in chunk) + ",")
    print("};")

generate_lut()
