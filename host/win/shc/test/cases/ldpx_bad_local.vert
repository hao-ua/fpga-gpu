// only uniform arrays take a run-time index: a local array must still be refused
attribute vec4 a_pos;
attribute float a_k;
void main()
{
    vec4 l[3];
    l[0] = a_pos; l[1] = a_pos; l[2] = a_pos;
    gl_Position = l[int(a_k)];
}
