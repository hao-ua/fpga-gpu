precision highp float;
varying vec4 v;
uniform sampler2D s;
uniform samplerCube cm;
void main() {
    vec4 a = clamp(v, 0.0, 1.0) + mix(v, v.wzyx, 0.5) + step(0.5, v) + smoothstep(0.1, 0.9, v);
    vec3 b = cross(v.xyz, v.wzy) + reflect(v.xyz, normalize(v.zyx)) + refract(v.xyz, normalize(v.yzx), 0.5);
    float c = atan(v.y, v.x) + atan(v.z) + asin(v.x) + acos(v.y) + tan(v.z) + radians(v.w) + degrees(v.x);
    float d = log(v.x) + log2(v.y) + exp2(v.z) + inversesqrt(v.w) + pow(v.x, v.y);
    vec2 e = min(v.xy, v.zw) + max(v.xy, 0.5) + mod(v.xy, 2.0) + faceforward(v.xy, v.zw, v.wz);
    vec4 f = texture2D(s, v.xy) + texture2D(s, v.xy, 1.0) + texture2DProj(s, v.xyz) + textureCube(cm, v.xyz);
    bvec4 g = lessThan(v, v.wzyx);
    bvec2 h = equal(v.xy, v.zw);
    float i = any(g) ? 1.0 : (all(h) ? 2.0 : float(not(h).x));
    gl_FragColor = a + vec4(b, c) + vec4(e, d, i) + f + gl_FragCoord * 0.001 + vec4(gl_FrontFacing ? 1.0 : 0.0);
}