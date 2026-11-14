#!/bin/bash
# gen_gpu_artifacts.sh — regenerate the FP24 / FP16 function tables and the arithmetic golden vectors in tools/gen/
# (git-ignored) with tools/float_ref.py: both boards' builds read fp24_{rcp_seed,rsqrt_seed,log2_lut,exp2_lut,sin_lut}.mem
# ($readmemh in float_sfu / sincos_sfu at synthesis; tools/build_board.tcl checks they exist), and the arithmetic benches
# (tb_float_alu, tb_float_sfu, tb_float_round, tb_sincos_sfu, tb_cvt, ...) read the *_vectors.hex.
# Pure Python 3, no numpy: run from WSL in the repository root,  bash tools/gen_gpu_artifacts.sh
# The other generated inputs come from their own tools: the board A / B draw goldens from tools/gen_a_goldens.sh and
# tools/b_ref.py, the shader cores' test programs per run from tools/run_vs2.ps1 / run_fs2.ps1 (tools/isa64.py), the
# MIG IP from tools/gen_mig.tcl.
set -e
cd "$(dirname "$0")/.."
mkdir -p tools/gen
python3 -u tools/float_ref.py > tools/gen/float_ref.txt
echo "float_ref: tables + golden vectors (log tools/gen/float_ref.txt)"
