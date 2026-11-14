#!/usr/bin/env python3
"""attr_conv_ref — test vectors for sim/tb_attr_conv.sv: every client attribute format of ES 2.0 §2.8 through
a_vtx_ref.conv (the model of src/rtl/attr_conv.sv). All 256 values of BYTE / UNSIGNED_BYTE normalized and not, the
extremes and random values of SHORT / UNSIGNED_SHORT / FIXED, FLOATs including zeros, denormals, Inf, NaN, the FP24
range edges and rounding ties at the 16-bit mantissa, and FP24 words taken as is.
  python3 tools/attr_conv_ref.py -> tools/gen/attr_conv_vectors.hex: one line per vector "ty nm raw32 expect24"."""
import os, sys, random, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a_vtx_ref as av

GEN = av.GEN


def main():
    rng = random.Random(42)
    vec = []                                              # (ty, nm, raw bytes)

    def add(ty, nm, raw): vec.append((ty, nm, raw))
    for c in range(256):
        for nm in (0, 1): add(av.T_BYTE, nm, bytes([c])); add(av.T_UBYTE, nm, bytes([c]))
    for c in (0, 1, 2, 0x7FFF, 0x8000, 0xFFFF, 0xFFFE, 0x8001, 0x4000, 0x0100, 0x00FF):
        for nm in (0, 1): add(av.T_SHORT, nm, c.to_bytes(2, 'little')); add(av.T_USHORT, nm, c.to_bytes(2, 'little'))
    for _ in range(600):
        c = rng.randrange(0, 65536)
        for nm in (0, 1): add(av.T_SHORT, nm, c.to_bytes(2, 'little')); add(av.T_USHORT, nm, c.to_bytes(2, 'little'))
    for c in (0, 1, -1, 0x10000, -0x10000, 0x7FFFFFFF, -0x80000000, 0x7FFFFFFE, 0x12345678, -0x12345678, 0xFFFF, 0x1FFFF, 0x3FFFF):
        add(av.T_FIXED, 0, (c & 0xFFFFFFFF).to_bytes(4, 'little'))
    for _ in range(600): add(av.T_FIXED, rng.randrange(2), rng.randrange(0, 1 << 32).to_bytes(4, 'little'))
    # FLOAT: specials, the FP24 range edges (2^-62 the smallest normal, 2^64 the first overflow), ties, random
    specials = [0.0, -0.0, 1.0, -1.0, 0.5, 2.0 ** -62, -(2.0 ** -62), 2.0 ** -63, 2.0 ** 63, 2.0 ** 64, -(2.0 ** 64), 1e-40, -1e-40, 1e38, 3.4028235e38]
    for x in specials: add(av.T_FLOAT, 0, struct.pack('<f', x))
    for w in (0x7F800000, 0xFF800000, 0x7FC00000, 0xFFC00000, 0x7F800001, 0x00000001, 0x80000001, 0x007FFFFF,
              0x3F800040, 0x3F8000C0, 0x3F80003F, 0x3F800041, 0x3F7FFFC0, 0x3F7FFFFF, 0x7F7FFFC0, 0x7F7FFF80, 0x0A800040, 0x0A000000, 0x0A7FFFFF):
        add(av.T_FLOAT, 0, w.to_bytes(4, 'little'))      # Inf, -Inf, NaNs, a signalling NaN, denormals, ties (mantissa[6:0] = 0x40) above even / odd
    for _ in range(1500):
        w = rng.randrange(0, 1 << 32)
        if rng.random() < 0.3: w = (w & 0xFFFFFF80) | 0x40      # a tie
        add(av.T_FLOAT, rng.randrange(2), w.to_bytes(4, 'little'))
    for _ in range(300): add(av.T_FLOAT, 0, struct.pack('<f', rng.uniform(-100, 100)))
    for _ in range(300): add(av.T_FP24, rng.randrange(2), rng.randrange(0, 1 << 32).to_bytes(4, 'little'))
    with open(f"{GEN}/attr_conv_vectors.hex", 'w') as f:
        for ty, nm, raw in vec:
            raw32 = int.from_bytes(raw.ljust(4, b'\0'), 'little')
            f.write(f"{ty:x} {nm:x} {raw32:08x} {av.conv(raw, ty, nm):06x}\n")
    print(f"attr_conv_vectors.hex: {len(vec)} vectors")


if __name__ == '__main__': main()
