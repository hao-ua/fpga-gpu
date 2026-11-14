// gl_FragColor written in part, read back, from a function, through gl_FragData, and as an out argument
precision mediump float;
varying vec4 v0;
uniform vec4 u;
void tint(float x) { gl_FragColor.r = x; }
void fill(out vec4 c) { c = u; }
void main()
{
    gl_FragColor.gba = vec3(0.0);
    tint(v0.x);
    gl_FragColor.xy = gl_FragColor.yx;
    if (v0.y > 0.5) gl_FragColor.g = 1.0;
    gl_FragColor *= u;
    gl_FragColor += gl_FragColor.wzyx;
    if (v0.z > 0.5) fill(gl_FragColor);
    gl_FragData[0].a = gl_FragColor.r;
}
