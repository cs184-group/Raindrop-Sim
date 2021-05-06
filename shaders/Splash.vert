#version 330

in vec4 vertex;
out vec2 v_texcoords;

uniform vec2 u_offset;
uniform mat4 u_model;
uniform mat4 u_view_projection;

void main() {
  v_texcoords = vec2(vertex.z + u_offset.x, vertex.w);
  vec4 view_v = u_view_projection * u_model * vec4(vertex.xy, 0, 1);
  gl_Position = view_v;
  //gl_Position = vec4(view_v.xy, 0, 1);
}