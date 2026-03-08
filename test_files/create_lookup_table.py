poly = 0x11021
table = []

for i in range(256):
    crc = i << 16

    while crc.bit_length() >= 17:
        div = poly << (crc.bit_length() - 17)
        crc = crc ^ div

    table.append(hex(crc))

str1 = "#include <stdint.h> \nconst uint16_t lookup_bytes[256] = {"
for i in table:
    str1 = str1 + i
    if i != table[-1]:
        str1 = str1 + ",\n"

str1 += "};"

with open("crc16_lookup.h", "w") as f:
    f.write(str1)
    
