attribute vec4 p;
uniform mat3 n;
varying mat3 m;
varying mat2 q;
varying vec2 after;
void main() { m = n; q[0] = p.xy; q[1] = p.zw; after = p.xy; gl_Position = p; }
