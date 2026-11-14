uniform mat4 CC_MVPMatrix;
uniform sampler2D CC_Texture0;
attribute vec4 a_position;
varying vec2 v;
float half_of(void) { return 0.5; }
void main(void) { v = a_position.xy * half_of(); gl_Position = CC_MVPMatrix * a_position; }
