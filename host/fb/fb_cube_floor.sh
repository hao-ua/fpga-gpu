#!/bin/bash
# fb_cube_floor.sh — the per-frame floor: one tiny cube (--scale 0.05, a few pixels), PRESENT without
# hold, 6 s each, in B's modes 0 (640 x 480), 1 (800 x 600), 3 (1280 x 800); prints one summary line per mode. Run from
# the repository root in WSL. The frame time of these runs is the fixed cost (B's band flush) a frame pays before any work.
cd "$(dirname "$0")/../.."
for m in 0 1 3; do
    timeout 120 python3 host/fb/fb_cube.py --cubes 1 --scale 0.05 --nohold --secs 6 --mode $m 2>&1 | grep -E '^fb_cube|^board A' | sed "s/^/mode $m: /"
done
