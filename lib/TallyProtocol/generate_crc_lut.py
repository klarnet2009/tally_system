#!/usr/bin/env python3
def generate_crc_table():
    poly = 0x07
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc = crc << 1
            crc &= 0xFF
        table.append(crc)

    print("static const uint8_t CRC8_LUT[256] = {")
    for i in range(0, 256, 8):
        row = ", ".join(f"0x{x:02X}" for x in table[i:i+8])
        if i + 8 < 256:
            row += ","
        print(f"  {row}")
    print("};")

if __name__ == '__main__':
    generate_crc_table()
