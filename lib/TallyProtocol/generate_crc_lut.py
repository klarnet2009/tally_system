def generate_crc8_ccitt_lut():
    polynomial = 0x07
    print("static const uint8_t CRC8_LUT[256] = {")
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ polynomial) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

        if i % 8 == 0:
            print("  ", end="")
        print(f"0x{crc:02X}, ", end="")
        if i % 8 == 7:
            print()
    print("};")

if __name__ == "__main__":
    generate_crc8_ccitt_lut()
