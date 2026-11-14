// a fragment shader keeps the constant-index rule: both compilers must refuse this
precision mediump float;
uniform vec4 u_c[8];
varying float v_k;
void main()
{
    gl_FragColor = u_c[int(v_k)];
}
