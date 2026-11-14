#!/usr/bin/env python3
"""mmcm_drp_calc.py — board B video modes.

Computes the MMCME2_ADV dynamic-reconfiguration (DRP) register values of every video mode and writes
src/rtl/disp/video_modes.svh: the DRP ROM (address, keep-mask, data per register, the 23-register
sequence of Xilinx XAPP888), the video timing of each mode and the static MMCM attributes.

The counter calculations are a port of XAPP888's mmcme2_drp_func.h as carried in Vivado's UNISIM
MMCME2_ADV simulation model (mmcm_divider, mmcm_phase, mmcm_calc, mmcm_frac_calc, round_frac); the lock
and loop-filter tables were extracted from the same model (case tables over the integer part of
CLKFBOUT_MULT_F). sim/tb_video_mmcm.sv checks every generated register against the DRP contents of a
UNISIM MMCME2_ADV instantiated statically with the same attributes, then reconfigures one MMCM through
all modes and measures its output periods.

Run from the repository root (WSL python3 is enough, no packages):  python3 tools/mmcm_drp_calc.py
"""
import os
import sys

CLKIN_MHZ = 200.0
VCO_MIN, VCO_MAX = 600.0, 1440.0      # XC7A200T -2 (DS181 MMCM_FVCOMIN / MMCM_FVCOMAX)
PFD_MIN, PFD_MAX = 10.0, 450.0        # MMCM_FPFDMIN / MMCM_FPFDMAX, -2

# id, name, CLKFBOUT_MULT_F, DIVCLK_DIVIDE, CLKOUT0_DIVIDE_F (serial = 5 x pixel), CLKOUT1_DIVIDE (pixel),
# horizontal (active, front porch, sync, back porch), vertical (same), sync polarities (1 = positive)
MODES = [
    dict(name="640x480@60 CEA/DMT", mult=20.125, div=4, o0=8, o1=40,
         h=(640, 16, 96, 48), v=(480, 10, 2, 33), hpol=0, vpol=0),
    dict(name="800x600@60 VESA DMT", mult=5.0, div=1, o0=5, o1=25,
         h=(800, 40, 128, 88), v=(600, 1, 4, 23), hpol=1, vpol=1),
    dict(name="1280x720@60 CEA-861 VIC 4", mult=37.125, div=10, o0=2, o1=10,
         h=(1280, 110, 40, 220), v=(720, 5, 5, 20), hpol=1, vpol=1),
    dict(name="1280x800@60 VESA CVT-RB", mult=35.5, div=10, o0=2, o1=10,
         h=(1280, 48, 32, 80), v=(800, 3, 6, 14), hpol=1, vpol=0),
    # VESA DMT 1024 x 768 @ 60 Hz, 65.000 MHz, negative syncs
    # (1344 x 806 total: 60.004 Hz); VCO 1300 MHz = 200 x 6.5, pixel / 20, serial / 4
    dict(name="1024x768@60 VESA DMT", mult=6.5, div=1, o0=4, o1=20,
         h=(1024, 24, 136, 160), v=(768, 3, 6, 29), hpol=0, vpol=0),
    # simulation only (disp_b / fb_cmd_b parameter SIM_MODE4): tiny frames on the 800x600 clocks (5) and on
    # the fractional 640x480 clocks with negative syncs (6), so benches can switch modes quickly
    dict(name="64x48 simulation test mode", mult=5.0, div=1, o0=5, o1=25,
         h=(64, 8, 8, 8), v=(48, 2, 2, 4), hpol=1, vpol=1),
    dict(name="80x40 simulation test mode", mult=20.125, div=4, o0=8, o1=40,
         h=(80, 8, 16, 8), v=(40, 2, 2, 4), hpol=0, vpol=0),
]
STATIC_MODE = 2      # the MMCM's static attributes = the fastest mode, so timing analysis covers every mode

# ---------------- lock and filter tables (UNISIM MMCME2_ADV model, index = integer CLKFBOUT_MULT 1..64) ----------------
# (lock_ref_dly, lock_fb_dly, lock_cnt, lock_sat_high, unlock_cnt)
LOCK = [(6,6,1000,1001,1),(6,6,1000,1001,1),(8,8,1000,1001,1),(11,11,1000,1001,1),(14,14,1000,1001,1),
        (17,17,1000,1001,1),(19,19,1000,1001,1),(22,22,1000,1001,1),(25,25,1000,1001,1),(28,28,1000,1001,1),
        (31,31,900,1001,1),(31,31,825,1001,1),(31,31,750,1001,1),(31,31,700,1001,1),(31,31,650,1001,1),
        (31,31,625,1001,1),(31,31,575,1001,1),(31,31,550,1001,1),(31,31,525,1001,1),(31,31,500,1001,1),
        (31,31,475,1001,1),(31,31,450,1001,1),(31,31,425,1001,1),(31,31,400,1001,1),(31,31,400,1001,1),
        (31,31,375,1001,1),(31,31,350,1001,1),(31,31,350,1001,1),(31,31,325,1001,1),(31,31,325,1001,1),
        (31,31,300,1001,1),(31,31,300,1001,1),(31,31,300,1001,1),(31,31,275,1001,1),(31,31,275,1001,1),
        (31,31,275,1001,1)] + [(31,31,250,1001,1)] * 28
# BANDWIDTH = "OPTIMIZED" (identical to the model's "HIGH" table): (charge pump, resistor); lfhf = 0
FILT_OPT = [(0b0010,0b1111),(0b0100,0b1111),(0b0101,0b1011),(0b0111,0b0111),(0b1101,0b0111),(0b1110,0b1011),
            (0b1110,0b1101),(0b1111,0b0011),(0b1110,0b0101),(0b1111,0b0101),(0b1111,0b1001),(0b1101,0b0001),
            (0b1111,0b1001),(0b1111,0b1001),(0b1111,0b1001),(0b1111,0b1001),(0b1111,0b0101),(0b1111,0b0101),
            (0b1100,0b0001),(0b1100,0b0001),(0b1100,0b0001),(0b0101,0b1100),(0b0101,0b1100),(0b0101,0b1100),
            (0b0101,0b1100)] + [(0b0011,0b0100)] * 16 + [(0b0010,0b1000)] * 5 + \
           [(0b0111,0b0001)] * 2 + [(0b0100,0b1100)] * 4 + [(0b0110,0b0001)] * 2 + [(0b0101,0b0110)] * 3 + \
           [(0b0010,0b0100)] * 4 + [(0b0100,0b1010)] + [(0b0011,0b1100)] * 2
assert len(LOCK) == 64 and len(FILT_OPT) == 64

# ---------------- XAPP888 counter calculations ----------------
FRAC = 10   # fixed-point fraction bits (MMCME2_ADV_FRAC_PRECISION)

def _bit1(v, k):
    """bit k of v in a [N:1] (1-based) Verilog vector, as the original functions index them"""
    return (v >> (k - 1)) & 1

def round_frac(dec, prec):
    return (dec + (1 << (FRAC - prec))) & 0xFFFFFFFF if _bit1(dec, FRAC - prec) else dec

def mmcm_divider(divide, duty):
    """divide 1..128, duty x 100000 -> {w_edge, no_count, high_time[5:0], low_time[5:0]}"""
    assert 0 < duty < 100000
    duty_fix = (duty << FRAC) // 100000
    if divide == 1:
        ht, we, lt, nc = 1, 0, 1, 1
    else:
        temp = round_frac((duty_fix * divide) & 0xFFFFFFFF, 1)
        ht = (temp >> FRAC) & 0x7F                  # temp[FRAC+7:FRAC+1]
        we = _bit1(temp, FRAC)                      # temp[FRAC]
        if ht == 0:
            ht, we = 1, 0
        if ht == divide:
            ht, we = divide - 1, 1
        lt = (divide - ht) & 0x7F
        nc = 0
    return (we << 13) | (nc << 12) | ((ht & 63) << 6) | (lt & 63)

def mmcm_phase(divide, phase):
    """phase in millidegrees; only 0 is used by the video modes (the signed paths are not ported)"""
    assert phase == 0
    return 0                                        # {mx = 0, phase_mux = 0, delay_time = 0}

def mmcm_calc(divide, phase, duty):
    dc = mmcm_divider(divide, duty)
    ph = mmcm_phase(divide, phase)
    upper = (((ph >> 9) & 3) << 8) | (((dc >> 12) & 3) << 6) | (ph & 63)
    lower = (((ph >> 6) & 7) << 13) | (dc & 0xFFF)
    return (upper << 16) | lower

def mmcm_frac_calc(divide, phase, duty, frac):
    """fractional counter (CLKOUT0 / CLKFBOUT); frac = fraction x 1000 in steps of 125"""
    assert phase == 0
    dfrac = frac // 125
    even_high = divide >> 1
    even_low = even_high
    odd = divide - even_high - even_low
    odd_and_frac = 8 * odd + dfrac
    lt_frac = even_high - (1 if odd_and_frac <= 9 else 0)
    ht_frac = even_low - (1 if odd_and_frac <= 8 else 0)
    pm_fall = ((odd & 0x7F) << 2) + (dfrac >> 1)
    wf_fall = 1 if ((2 <= odd_and_frac <= 9) or (dfrac == 1 and divide == 2)) else 0
    wf_rise = 1 if (1 <= odd_and_frac <= 8) else 0
    a_per = 8 * divide + dfrac
    a_phase = ((phase + 10) * a_per) // 360000
    pm_rise_frac = 0 if (a_phase & 0xFF) == 0 else (a_phase & 7)
    dt = (((phase + 10) * a_per // 8) // 360000) & 0xFF
    pm_rise_f = pm_rise_frac - 8 if pm_rise_frac >= 8 else pm_rise_frac
    pm_fall_f = (pm_fall + pm_rise_frac) & 7
    dc = mmcm_divider(divide, duty)
    ph = mmcm_phase(divide, phase)
    top = ((pm_fall_f & 7) << 1) | wf_fall
    upper = (dfrac << 12) | (1 << 11) | (wf_rise << 10) | (((ph >> 9) & 3) << 8) | (((dc >> 12) & 3) << 6) | (dt & 63)
    lower = ((pm_rise_f & 7) << 13) | ((ht_frac & 63) << 6) | (lt_frac & 63)
    return (top << 32) | (upper << 16) | lower

def counter(div):
    """attr_to_mc(): 50 % duty, phase 0; fractional when the value has a fraction"""
    div_int = int(div)
    frac_en = (div - div_int) > 0
    div_rnd = int((div + 0.0625) * 8.0) / 8.0
    div_int = int(div_rnd)
    div_frac = div_rnd - div_int
    if frac_en:
        return mmcm_frac_calc(div_int, 0, 50000, int(round(div_frac * 1000))), True
    return mmcm_calc(div_int, 0, 50000), False

# ---------------- register sequence (XAPP888 mmcme2_drp.v order and keep-masks) ----------------
def drp_regs(m):
    c0, c0f = counter(m["o0"])
    c1, _ = counter(m["o1"])
    cu, _ = counter(1)                                # CLKOUT2..6 unused: divide 1 (attribute default)
    fb, fbf = counter(m["mult"])
    dv, _ = counter(m["div"])
    lk = LOCK[int(m["mult"]) - 1]
    cp, res = FILT_OPT[int(m["mult"]) - 1]
    filt = (cp << 6) | (res << 2) | 0                 # {cp[3:0], res[3:0], lfhf[1:0]}
    def b(v, hi, lo):
        return (v >> lo) & ((1 << (hi - lo + 1)) - 1)
    c5_hi = (cu >> 16) & 0xFFFF
    c6_hi = (cu >> 16) & 0xFFFF
    if c0f:
        c5_hi = (b(cu, 31, 30) << 14) | (b(c0, 35, 32) << 10) | b(cu, 25, 16)
    if fbf:
        c6_hi = (b(cu, 31, 30) << 14) | (b(fb, 35, 32) << 10) | b(cu, 25, 16)
    lock_cnt = lk[2]
    regs = [
        (0x28, 0x0000, 0xFFFF),                                        # power
        (0x08, 0x1000, c0 & 0xFFFF), (0x09, 0x8000, (c0 >> 16) & 0xFFFF),   # CLKOUT0 (serial)
        (0x0A, 0x1000, c1 & 0xFFFF), (0x0B, 0xFC00, (c1 >> 16) & 0xFFFF),   # CLKOUT1 (pixel)
        (0x0C, 0x1000, cu & 0xFFFF), (0x0D, 0xFC00, (cu >> 16) & 0xFFFF),   # CLKOUT2
        (0x0E, 0x1000, cu & 0xFFFF), (0x0F, 0xFC00, (cu >> 16) & 0xFFFF),   # CLKOUT3
        (0x10, 0x1000, cu & 0xFFFF), (0x11, 0xFC00, (cu >> 16) & 0xFFFF),   # CLKOUT4
        (0x06, 0x1000, cu & 0xFFFF), (0x07, 0xC000, c5_hi),                 # CLKOUT5 (+ CLKOUT0 fraction bits)
        (0x12, 0x1000, cu & 0xFFFF), (0x13, 0xC000, c6_hi),                 # CLKOUT6 (+ CLKFBOUT fraction bits)
        (0x16, 0xC000, (b(dv, 23, 22) << 12) | b(dv, 11, 0)),             # DIVCLK
        (0x14, 0x1000, fb & 0xFFFF), (0x15, 0x8000, (fb >> 16) & 0xFFFF),   # CLKFBOUT
        (0x18, 0xFC00, lock_cnt & 0x3FF),                                  # lock
        (0x19, 0x8000, ((lk[1] & 31) << 10) | (lk[4] & 0x3FF)),
        (0x1A, 0x8000, ((lk[0] & 31) << 10) | (lk[3] & 0x3FF)),
        (0x4E, 0x66FF, (b(filt, 9, 9) << 15) | (b(filt, 8, 7) << 11) | (b(filt, 6, 6) << 8)),   # filter
        (0x4F, 0x666F, (b(filt, 5, 5) << 15) | (b(filt, 4, 3) << 11) | (b(filt, 2, 1) << 7) | (b(filt, 0, 0) << 4)),
    ]
    for a, k, d in regs:
        assert d & k == 0, (hex(a), hex(k), hex(d))
    return regs

def check_mode(i, m):
    vco = CLKIN_MHZ * m["mult"] / m["div"]
    pfd = CLKIN_MHZ / m["div"]
    ser = vco / m["o0"]
    pix = vco / m["o1"]
    assert VCO_MIN <= vco <= VCO_MAX, f"mode {i}: VCO {vco}"
    assert PFD_MIN <= pfd <= PFD_MAX, f"mode {i}: PFD {pfd}"
    assert abs(ser - 5 * pix) < 1e-9, f"mode {i}: serial clock is not 5 x pixel"
    assert (m["mult"] * 8) == int(m["mult"] * 8) and 2.0 <= m["mult"] <= 64.0
    assert m["h"][0] % 8 == 0 and m["h"][0] <= 1280, "active width: multiple of 8, at most 1280"
    return vco, pix, ser

def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "src", "rtl", "disp", "video_modes.svh")
    L = []
    L.append("// video_modes.svh — GENERATED by tools/mmcm_drp_calc.py, do not edit.")
    L.append("// Board B video modes: MMCME2_ADV DRP register sequence")
    L.append("// (XAPP888 order and keep-masks, new = (old & mask) | data), video timing, static MMCM attributes.")
    L.append("// CLKIN 200 MHz; CLKOUT0 = serial clock (5 x pixel, DDR 10:1 TMDS), CLKOUT1 = pixel clock.")
    for i, m in enumerate(MODES):
        vco, pix, ser = check_mode(i, m)
        L.append(f"//   mode {i}: {m['name']:28s} M {m['mult']:7.3f} D {m['div']:2d} O0 {m['o0']:3d} O1 {m['o1']:3d}"
                 f"  VCO {vco:8.3f}  pixel {pix:7.3f} MHz  serial {ser:7.3f} MHz")
    L.append("// Module-scope declarations: include inside each module that uses them (no include guard).")
    L.append(f"localparam int VM_NUM_MODES   = {len(MODES)};       // ids 0..4 = hardware modes, 5..6 = simulation test modes")
    L.append(f"localparam int VM_DRP_REGS    = 23;")
    L.append(f"localparam int VM_STATIC_MODE = {STATIC_MODE};")
    s = MODES[STATIC_MODE]
    L.append(f"localparam real VM_STATIC_MULT_F = {s['mult']:.3f};")
    L.append(f"localparam int  VM_STATIC_DIVCLK = {s['div']};")
    L.append(f"localparam real VM_STATIC_O0_F   = {float(s['o0']):.3f};")
    L.append(f"localparam int  VM_STATIC_O1     = {s['o1']};")
    L.append("// static attributes per mode (simulation golden instances in sim/tb_video_mmcm.sv)")
    L.append("function automatic real vm_mult_f(input int m);")
    L.append("    case (m)")
    for i, m in enumerate(MODES):
        L.append(f"        {i}: vm_mult_f = {m['mult']:.3f};")
    L.append("        default: vm_mult_f = 0.0;")
    L.append("    endcase")
    L.append("endfunction")
    for fn, key in (("vm_divclk", "div"), ("vm_o0", "o0"), ("vm_o1", "o1")):
        L.append(f"function automatic int {fn}(input int m);")
        L.append("    case (m)")
        for i, m in enumerate(MODES):
            L.append(f"        {i}: {fn} = {m[key]};")
        L.append(f"        default: {fn} = 0;")
        L.append("    endcase")
        L.append("endfunction")
    L.append("// pixel clock in kHz (status / bench check)")
    L.append("function automatic [31:0] vm_pix_khz(input [2:0] m);")
    L.append("    case (m)")
    for i, m in enumerate(MODES):
        _, pix, _ = check_mode(i, m)
        L.append(f"        3'd{i}: vm_pix_khz = 32'd{int(round(pix * 1000))};")
    L.append("        default: vm_pix_khz = 32'd0;")
    L.append("    endcase")
    L.append("endfunction")
    L.append("// timing: {h_active, h_fp, h_sync, h_bp, v_active, v_fp, v_sync, v_bp} 12 bits each, {hpol, vpol}")
    L.append("function automatic [97:0] vm_timing(input [2:0] m);")
    L.append("    case (m)")
    for i, m in enumerate(MODES):
        h, v = m["h"], m["v"]
        fields = ", ".join(f"12'd{x}" for x in (h + v))
        L.append(f"        3'd{i}: vm_timing = {{{fields}, 1'b{m['hpol']}, 1'b{m['vpol']}}};")
    L.append("        default: vm_timing = '0;")
    L.append("    endcase")
    L.append("endfunction")
    L.append("// DRP sequence entry: {daddr[6:0], keep_mask[15:0], data[15:0]}")
    L.append("function automatic [38:0] vm_drp(input [2:0] m, input [4:0] k);")
    L.append("    case ({m, k})")
    for i, m in enumerate(MODES):
        for k, (a, msk, d) in enumerate(drp_regs(m)):
            L.append(f"        {{3'd{i}, 5'd{k:2d}}}: vm_drp = {{7'h{a:02X}, 16'h{msk:04X}, 16'h{d:04X}}};")
    L.append("        default: vm_drp = {7'h28, 16'hFFFF, 16'h0000};   // power register, unchanged")
    L.append("    endcase")
    L.append("endfunction")
    text = "\n".join(L) + "\n"
    with open(out, "w", newline="\n") as f:
        f.write(text)
    print(f"wrote {out}")
    for i, m in enumerate(MODES):
        print(f"mode {i}: " + " ".join(f"{a:02X}:{d:04X}" for a, _, d in drp_regs(m)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
