#!/bin/bash
# gen_a_goldens.sh — regenerate board A's draw / record goldens (tools/gen/av_<seed>_*, as_<seed>_*) with the draw
# options each seed was made with (tb_a_geom seeds 1, 2, 3, 6, 7, 8, 9; tb_a_cmd 1, 2, 3, 8, 9;
# tb_board_a seed 1). Run from the repository root (WSL): bash tools/gen_a_goldens.sh
set -e
python3 tools/a_setup_ref.py --check
python3 tools/a_setup_ref.py 1
python3 tools/a_setup_ref.py 2 --mode strip
python3 tools/a_setup_ref.py 3 --mode fan --nv 30 --nidx 40
python3 tools/a_setup_ref.py 6 --nv 200 --nidx 600
python3 tools/a_setup_ref.py 7 --mode strip --nv 150 --nidx 400 --cull 0
python3 tools/a_setup_ref.py 8 --mode fan --nv 300 --nidx 300
python3 tools/a_setup_ref.py 9 --nv 120 --nidx 300 --fsmask ff
# small targets for the render-side bench (tb_a_render: the whole stream is compared beat for beat)
python3 tools/a_setup_ref.py 11 --nv 30 --nidx 60 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 12 --mode strip --nv 40 --nidx 90 --wid 96 --hgt 48 --fsmask ff
# points (gl_PointSize from the program) and lines of a width (small targets). The seeds that also run through
# tb_two_boards3d (13, 15, as 11, 12) need a varying mask that covers varyings 0 and 1: B's test fragment program
# b_fs_two reads them, and a varying the mask omits is undefined on the hardware (fbproto.vs_state: fsmask = the
# fragment shader's read varyings) — b_ref.py aframe refuses such a draw.
python3 tools/a_setup_ref.py 13 --mode points --nv 30 --nidx 40 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 14 --mode lines --nv 30 --nidx 40 --wid 128 --hgt 64 --lw 1
python3 tools/a_setup_ref.py 15 --mode lstrip --nv 20 --nidx 20 --wid 96 --hgt 48 --lw 3 --fsmask 0b
python3 tools/a_setup_ref.py 16 --mode lloop --nv 20 --nidx 12 --wid 96 --hgt 48 --lw 2
# an unaligned index-buffer offset (u16 at an odd index, u8 at byte 3) and CULL FRONT_AND_BACK;
# 17 / 18 only through the command path (tb_a_cmd, tb_a_render: the DRAW carries the byte offset), 19 everywhere
python3 tools/a_setup_ref.py 17 --nv 30 --nidx 60 --ioff 1 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 18 --mode strip --u8 --nv 30 --nidx 40 --ioff 3 --wid 96 --hgt 48
python3 tools/a_setup_ref.py 19 --nv 30 --nidx 60 --cull 3 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 20 --nv 30 --nidx 60 --depth 0.2 0.9 --wid 128 --hgt 64        # glDepthRangef(0.2, 0.9)
# client attribute formats (BYTE .. FLOAT, normalized, byte strides / offsets with random
# padding — a_vtx_ref --types, the layout printed per seed), a DrawArrays range beyond 65535 (--first), the converter's vectors
python3 tools/attr_conv_ref.py
python3 tools/a_setup_ref.py 21 --types --nv 40 --nidx 90 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 22 --types --mode strip --nv 60 --nidx 120 --wid 128 --hgt 64 --fsmask ff
python3 tools/a_setup_ref.py 23 --types --mode fan --u8 --nv 50 --nidx 60 --ioff 2 --wid 96 --hgt 48
python3 tools/a_setup_ref.py 24 --types --arrays --first 70000 --nv 30 --nidx 60 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 25 --types --mode points --nv 30 --nidx 40 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 26 --types --mode lstrip --nv 25 --nidx 25 --wid 96 --hgt 48 --lw 2 --fsmask 0b
# draw overlap: several draws back to back with their own uniforms / draw id / cull — command path only
# (tb_a_cmd, tb_a_render; the as_<seed>_state.mem / av_<seed>_expect.mem files describe draw 0 alone)
python3 tools/a_setup_ref.py 27 --ndraw 3 --nv 30 --nidx 60 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 28 --ndraw 4 --types --mode strip --u8 --nv 40 --nidx 50 --wid 96 --hgt 48 --fsmask ff
python3 tools/a_setup_ref.py 29 --ndraw 5 --mode points --nv 20 --nidx 24 --wid 128 --hgt 64 --fsmask 27
# the x / y guard band (vertices far outside the view: --far), the clipper's vectors with the guard planes,
# lines against the guard planes, points of odd sizes near the top of a tall target (the 18-bit centre)
python3 tools/a_setup_ref.py --clipvec
python3 tools/a_setup_ref.py 30 --far --nv 40 --nidx 90 --wid 128 --hgt 64
python3 tools/a_setup_ref.py 31 --far --mode strip --nv 60 --nidx 120 --wid 96 --hgt 48 --fsmask ff
python3 tools/a_setup_ref.py 32 --far --mode lines --nv 40 --nidx 60 --wid 128 --hgt 64 --lw 3
python3 tools/a_setup_ref.py 33 --far --mode points --nv 300 --nidx 400 --wid 64 --hgt 2048 --fsmask 27
# overdraw: deep overlapping draws, several
# draws (35, 36) and a strip, points — the walk's many visits per band (tb_a_render) and b_ref aframe's two-board frames
python3 tools/a_setup_ref.py 34 --nv 60 --nidx 180 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 35 --ndraw 3 --nv 40 --nidx 120 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 36 --ndraw 2 --mode strip --nv 60 --nidx 120 --wid 96 --hgt 48 --fsmask ff
python3 tools/a_setup_ref.py 37 --nv 60 --nidx 180 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 38 --mode points --nv 40 --nidx 60 --wid 128 --hgt 64 --fsmask 27
# --huge = vertices 60 x, so most triangles have |area| >= 2^30 and the edge shift
# s > 0 — a_setup's a2s must keep the sign of a negative area in that case (an unsigned ?: arm makes the >>> logical)
python3 tools/a_setup_ref.py 39 --huge --nv 40 --nidx 150 --wid 128 --hgt 64 --fsmask 27
# targets wider than 640 — the band height's rule (8 rows up to 1024 wide, else 4; b_ref.band_lines): 41 the
# walk's profile draw (800 x 64, many visits; tb_a_render +FASTSINK +RNPROF), 42 / 43 small draws either side of the limit
# (also b_ref.py aframe for tb_two_boards3d)
python3 tools/a_setup_ref.py 41 --nv 120 --nidx 360 --wid 800 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 42 --nv 30 --nidx 60 --wid 800 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 43 --nv 30 --nidx 60 --wid 1040 --hgt 48 --fsmask 27
# the draw's rectangle (--sc: its scissor box, VS_STATE's 15-word form) — board A drops what has no pixel in it,
# bins by its rows and walks inside it. 44 triangles, 45 points, 46 lines of a width (all three also b_ref.py aframe for
# tb_two_boards3d: B's scissor is the same box); 47 four draws alternating with and without it (both forms of the command,
# the per-draw state queue: board A's benches only); 48 a strip through the top rows alone (the band range); 49 a box that
# reaches beyond the target (cut to it); 50 a one-pixel box; 51 a wide target; 52 an empty box (nothing is drawn)
python3 tools/a_setup_ref.py 44 --nv 60 --nidx 180 --wid 128 --hgt 64 --fsmask 27 --sc 37 13 91 50
python3 tools/a_setup_ref.py 45 --mode points --nv 40 --nidx 60 --wid 128 --hgt 64 --fsmask 27 --sc 20 9 100 41
python3 tools/a_setup_ref.py 46 --mode lines --nv 40 --nidx 60 --wid 128 --hgt 64 --lw 3 --fsmask 27 --sc 33 17 96 47
python3 tools/a_setup_ref.py 47 --ndraw 4 --scalt --nv 40 --nidx 120 --wid 128 --hgt 64 --fsmask 27 --sc 16 8 65 33
python3 tools/a_setup_ref.py 48 --mode strip --nv 60 --nidx 120 --wid 96 --hgt 48 --fsmask ff --sc 0 0 96 17
python3 tools/a_setup_ref.py 49 --far --nv 40 --nidx 90 --wid 128 --hgt 64 --fsmask 27 --sc 100 40 300 200
python3 tools/a_setup_ref.py 50 --mode lstrip --nv 20 --nidx 20 --wid 96 --hgt 48 --lw 2 --fsmask 0b --sc 11 5 12 6
python3 tools/a_setup_ref.py 51 --nv 30 --nidx 60 --wid 800 --hgt 64 --fsmask 27 --sc 301 3 517 60
python3 tools/a_setup_ref.py 52 --nv 30 --nidx 60 --wid 128 --hgt 64 --fsmask 27 --sc 50 20 50 40
# larger draws, where the box REJECTS: 53 a 10 x 8 box in 200 triangles (202 records without it, 120 with; 31 626 quads -> 942),
# 54 points (290 -> 197 records), 55 lines of width 3 (97 -> 83) — the draws a mutant of a_setup's bounds fails on
python3 tools/a_setup_ref.py 53 --nv 200 --nidx 600 --wid 128 --hgt 64 --fsmask 27 --sc 60 28 70 36
python3 tools/a_setup_ref.py 54 --mode points --nv 300 --nidx 400 --wid 128 --hgt 64 --fsmask 27 --sc 20 9 100 41
python3 tools/a_setup_ref.py 55 --mode lines --nv 120 --nidx 200 --wid 128 --hgt 64 --lw 3 --fsmask 27 --sc 33 17 96 47
# the clip volume 0 <= z <= w (--z0, VS_STATE word 4 bit 24: depth kept reversed by the libraries, far at z = 0):
# plane 1 is z >= 0. 56 triangles (about half the vertices have z < 0), 57 a strip with every varying, 58 points, 59 lines
# of a width, 60 three draws
python3 tools/a_setup_ref.py 56 --z0 --nv 60 --nidx 180 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 57 --z0 --mode strip --nv 60 --nidx 120 --wid 96 --hgt 48 --fsmask ff
python3 tools/a_setup_ref.py 58 --z0 --mode points --nv 60 --nidx 80 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_setup_ref.py 59 --z0 --mode lines --nv 40 --nidx 60 --wid 128 --hgt 64 --lw 3 --fsmask 27
python3 tools/a_setup_ref.py 60 --z0 --ndraw 3 --nv 40 --nidx 120 --wid 128 --hgt 64 --fsmask 27
python3 tools/a_raster_ref.py --check
for s in 1 2 3 6 7 8 9 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60; do python3 tools/a_raster_ref.py $s; done     # the render stream of each draw (a_render)
