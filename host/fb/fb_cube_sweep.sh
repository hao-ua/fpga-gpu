#!/bin/bash
# fb_cube_sweep.sh — the performance sweep of host/fb/fb_cube.py: cube counts x draw modes, PRESENT without hold (frames
# as fast as the GPU renders them), 6 s each, ALWAYS at 800 x 600 (mode 1: B's per-frame floor depends on the mode, so
# a sweep taken in whatever mode B was left in is not comparable). MODE=<n> overrides. Prints one summary line per run. Run from the repository root in WSL.
cd "$(dirname "$0")/../.."
MODE=${MODE:-1}
echo "===== one draw, mode $MODE"
for n in 1 16 64 256 1024; do
    timeout 120 python3 host/fb/fb_cube.py --cubes $n --nohold --secs 6 --mode $MODE 2>&1 | grep '^fb_cube'
done
echo "===== a draw per cube, mode $MODE"
for n in 1 16 64 256; do
    timeout 120 python3 host/fb/fb_cube.py --cubes $n --draws --nohold --secs 6 --mode $MODE 2>&1 | grep '^fb_cube'
done
