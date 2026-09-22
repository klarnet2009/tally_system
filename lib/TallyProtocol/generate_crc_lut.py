#!/usr/bin/env python3

def generate_crc_lut():
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

    print("static const uint8_t crc8_lut[256] = {")
    for i in range(0, 256, 12):
        chunk = lut[i:i+12]
        line = "  " + ", ".join(f"0x{b:02X}" for b in chunk)
        if i + 12 < 256:
            line += ","
        print(line)
    print("};")

if __name__ == "__main__":
    generate_crc_lut()
