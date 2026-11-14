// g2_render: a clear inside a pass, drawn as a quad. The colour is the clear's own value per channel (g2_render.c
// clear_quad: c16 / ONE, which board B's conversion turns back into exactly that c16); with the colour writes off
// (a depth / stencil clear) it is not used
precision mediump float;
uniform vec4 uC;
void main() { gl_FragColor = uC; }
