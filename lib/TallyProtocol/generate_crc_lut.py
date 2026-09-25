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

    # Format as C array
    output = "static const uint8_t CRC8_LUT[256] = {\n"
    for i in range(0, 256, 8):
        line = "  " + ", ".join(f"0x{val:02X}" for val in lut[i:i+8]) + ","
        output += line + "\n"
    output += "};"
    return output

if __name__ == "__main__":
    print(generate_crc8_ccitt_lut())
