attribute vec4 a;
varying vec4 v;
uniform mat4 m;
vec4 p;
void place() { p = m * a; }
void main() {
    place();
    v = a;
    gl_Position = p;
}
