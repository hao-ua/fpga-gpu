precision mediump float;
varying vec4 v0;
uniform sampler2D plain;
struct T { float k; sampler2D t; };
struct S { float a; sampler2D tex; T inner; samplerCube cube[2]; };
uniform S s;
uniform S arr[2];
void main()
{
    vec4 c = texture2D(plain, v0.xy) + texture2D(s.tex, v0.xy) * s.a + texture2D(s.inner.t, v0.xy);
    c += textureCube(s.cube[1], v0.xyz) + texture2D(arr[1].tex, v0.xy) + texture2D(arr[0].inner.t, v0.xy) * arr[1].inner.k;
    gl_FragColor = c;
}
