def generate_crc_table():
    print("static const uint8_t CRC8_LUT[256] = {")
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        if i % 8 == 0:
            print("  ", end="")
        print(f"0x{crc:02x}, ", end="")
        if i % 8 == 7:
            print()
    print("};")

if __name__ == "__main__":
    generate_crc_table()
