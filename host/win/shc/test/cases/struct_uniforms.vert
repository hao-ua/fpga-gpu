precision highp float;
struct Light { vec3 pos; float power; mat3 basis; };
struct Scene { Light lights[3]; vec4 ambient; float k[2]; };
uniform Scene scene;
uniform Light extra[2];
uniform mat4 mvp;
attribute vec4 a_pos;
attribute vec3 a_n;
varying vec3 v_c;
void main() {
    vec3 c = scene.ambient.rgb * scene.k[1];
    for (int i = 0; i < 3; i++) {
        vec3 l = scene.lights[i].basis * (scene.lights[i].pos - a_pos.xyz);
        c += max(dot(normalize(l), a_n), 0.0) * scene.lights[i].power;
    }
    c += extra[1].pos * extra[0].power;
    v_c = c;
    gl_Position = mvp * a_pos;
}