precision mediump float;
varying mat2 m[2];
varying mat3 n[2];
varying vec4 w;
void main() {
    mat2 t = m[1];
    gl_FragColor = vec4(m[0][0], t[1][1] + n[1][2].x, n[0][1].y) * w;
}
