#version 330

in vec2 v_texcoords;
out vec4 out_color;

uniform float opacity;
uniform sampler2DRect u_texture_7;

void main() {
  vec4 color = texture(u_texture_7, v_texcoords);
  if (color.a <= 0.1) {
  	discard;
  } else {
    out_color = color * vec4(1, 1, 1, opacity);
  }
}