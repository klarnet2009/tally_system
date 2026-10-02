
def generate_crc_table():
    poly = 0x07
    crc_table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        crc_table.append(crc)

    print("const uint8_t CRC_LUT[256] = {")
    for i in range(0, 256, 8):
        row = crc_table[i:i+8]
        print("    " + ", ".join(f"0x{val:02X}" for val in row) + ",")
    print("};")

if __name__ == "__main__":
    generate_crc_table()
