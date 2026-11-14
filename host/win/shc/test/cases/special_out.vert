// gl_Position and gl_PointSize written in part and read back
attribute vec4 a_p;
varying vec4 v0;
void flip(inout vec4 p) { p.y = -p.y; }
void main()
{
    gl_Position = a_p;
    gl_Position.z = 2.0 * gl_Position.z - gl_Position.w;
    flip(gl_Position);
    gl_PointSize = a_p.x * 4.0;
    v0 = gl_Position * gl_PointSize;
}
