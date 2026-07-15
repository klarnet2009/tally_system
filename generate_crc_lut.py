def generate_table():
    poly = 0x07
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)

    # Print the table in C format
    print("static const uint8_t crc8_lut[256] = {")
    for i in range(0, 256, 8):
        print("    " + ", ".join(f"0x{x:02X}" for x in table[i:i+8]) + ",")
    print("};")

generate_table()
