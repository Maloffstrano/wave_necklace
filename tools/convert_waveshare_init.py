import re, sys
def parse(path, name):
    src = open(path).read()
    body = re.search(r'%s\[\]\s*=\s*\{(.*?)\n\};' % name, src, re.S).group(1)
    body = "\n".join(l for l in body.splitlines() if not l.strip().startswith("//"))
    out = []
    for m in re.finditer(r'\{\s*(0x[0-9A-Fa-f]+)\s*,\s*\(uint8_t\s*\[\]\)\s*\{([^}]*)\}\s*,\s*(\d+)\s*,\s*(\d+)\s*\}', body):
        cmd, data, n, d = m.group(1), [x.strip() for x in m.group(2).split(',') if x.strip()], int(m.group(3)), int(m.group(4))
        out.append((cmd, data[:n], d))
    return out
def emit(cmds, cname):
    L = ["static const uint8_t %s[] = {" % cname, "    BEGIN_WRITE,",
         "    WRITE_C8_D8, 0x3A, 0x55,  // COLMOD 16-bit (sent first by Waveshare's esp_lcd driver)",
         "    WRITE_C8_D8, 0x36, 0x00,  // MADCTL default"]
    for cmd, data, d in cmds:
        c = cmd.upper().replace("0X", "0x")
        if len(data) == 0: L.append("    WRITE_COMMAND_8, %s," % c)
        elif len(data) == 1: L.append("    WRITE_C8_D8, %s, %s," % (c, data[0].upper().replace("0X","0x")))
        else: L.append("    WRITE_COMMAND_8, %s, WRITE_BYTES, %d, %s," % (c, len(data), ", ".join(x.upper().replace("0X","0x") for x in data)))
        if d:
            L += ["    END_WRITE,", "    DELAY, %d," % d, "    BEGIN_WRITE,"]
    L += ["    END_WRITE};", ""]
    return "\n".join(L), len(cmds)
# Usage: python3 -I tools/convert_waveshare_init.py <demo>/esp_lcd_st77916.c <demo>/Display_ST77916.cpp > libraries/NecklaceBoard/src/st77916_waveshare_init.h
drv, cpp = sys.argv[1], sys.argv[2]
a, na = emit(parse(drv, "vendor_specific_init_default"), "waveshare_185_init_rev1")
b, nb = emit(parse(cpp, "vendor_specific_init_new"), "waveshare_185_init_rev2")
print("// Generated from Waveshare ESP32-S3-Touch-LCD-1.85 demo (esp_lcd_st77916.c / Display_ST77916.cpp)")
print("// rev1: panel register 0x04 reads 00 7F 7F 7F (%d cmds)  rev2: reads 00 02 7F 7F (%d cmds)" % (na, nb))
print("#pragma once\n#include <Arduino_GFX_Library.h>\n")
print(a); print(b)
