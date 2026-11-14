// licm_pressure.frag — 40 vec4 values a 40-pass loop works out from uniforms, each the same every pass: hoisted out of the loop
// (glslc.licm) all 40 stay live across it, more than the 31 registers; inside it each lives a line. The compile must not
// fail for the hoist: a register spill retries without it (compile_stage / g2_shc.c compile_opts, 2026-11-13, after TDM's Seascape).
precision mediump float;
varying vec4 v0;
uniform vec4 u[40];

void main()
{
    vec4 s = v0;
    for (int i = 0; i < 40; i++) {
        s = s * 0.5 + (u[0] * 1.5 + 0.25);
        s = s * 0.5 + (u[1] * 1.5 + 0.25);
        s = s * 0.5 + (u[2] * 1.5 + 0.25);
        s = s * 0.5 + (u[3] * 1.5 + 0.25);
        s = s * 0.5 + (u[4] * 1.5 + 0.25);
        s = s * 0.5 + (u[5] * 1.5 + 0.25);
        s = s * 0.5 + (u[6] * 1.5 + 0.25);
        s = s * 0.5 + (u[7] * 1.5 + 0.25);
        s = s * 0.5 + (u[8] * 1.5 + 0.25);
        s = s * 0.5 + (u[9] * 1.5 + 0.25);
        s = s * 0.5 + (u[10] * 1.5 + 0.25);
        s = s * 0.5 + (u[11] * 1.5 + 0.25);
        s = s * 0.5 + (u[12] * 1.5 + 0.25);
        s = s * 0.5 + (u[13] * 1.5 + 0.25);
        s = s * 0.5 + (u[14] * 1.5 + 0.25);
        s = s * 0.5 + (u[15] * 1.5 + 0.25);
        s = s * 0.5 + (u[16] * 1.5 + 0.25);
        s = s * 0.5 + (u[17] * 1.5 + 0.25);
        s = s * 0.5 + (u[18] * 1.5 + 0.25);
        s = s * 0.5 + (u[19] * 1.5 + 0.25);
        s = s * 0.5 + (u[20] * 1.5 + 0.25);
        s = s * 0.5 + (u[21] * 1.5 + 0.25);
        s = s * 0.5 + (u[22] * 1.5 + 0.25);
        s = s * 0.5 + (u[23] * 1.5 + 0.25);
        s = s * 0.5 + (u[24] * 1.5 + 0.25);
        s = s * 0.5 + (u[25] * 1.5 + 0.25);
        s = s * 0.5 + (u[26] * 1.5 + 0.25);
        s = s * 0.5 + (u[27] * 1.5 + 0.25);
        s = s * 0.5 + (u[28] * 1.5 + 0.25);
        s = s * 0.5 + (u[29] * 1.5 + 0.25);
        s = s * 0.5 + (u[30] * 1.5 + 0.25);
        s = s * 0.5 + (u[31] * 1.5 + 0.25);
        s = s * 0.5 + (u[32] * 1.5 + 0.25);
        s = s * 0.5 + (u[33] * 1.5 + 0.25);
        s = s * 0.5 + (u[34] * 1.5 + 0.25);
        s = s * 0.5 + (u[35] * 1.5 + 0.25);
        s = s * 0.5 + (u[36] * 1.5 + 0.25);
        s = s * 0.5 + (u[37] * 1.5 + 0.25);
        s = s * 0.5 + (u[38] * 1.5 + 0.25);
        s = s * 0.5 + (u[39] * 1.5 + 0.25);
        if (s.x > 100.0) break;
    }
    gl_FragColor = s;
}
