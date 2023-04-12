#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float helper(vec2 uv) {
    // You may want to use this helper function...
    vec4 temp = texture(u_texture_2, uv);
    return temp.r;
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  // out_color.a = 1;

  // constructing TBN matrix
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n, t);
  mat3 tbn = mat3(t, b, n);

  // compute dU, dV, n0, nd
  float w = u_texture_2_size[0];
  float h = u_texture_2_size[1];
  float dU = 10 * (helper(v_uv + vec2(1.0/w, 0.0)) - helper(v_uv)) * u_normal_scaling * u_height_scaling;
  float dV = 10 * (helper(v_uv + vec2(0.0, 1.0/h)) - helper(v_uv)) * u_normal_scaling * u_height_scaling;
  vec3 n0 = normalize(vec3(-dU, -dV, 1));
  vec3 nd = tbn * n0;
  
  // Define Phong params
  float ka = 0.1;
  vec3 kd = vec3(1.0);
  float ks = 0.8;
  vec3 Ia = vec3(0.1);
  float p = 50.0;
  
  // Phong Shader
  vec3 v = u_cam_pos - v_position.xyz;
  vec3 l = u_light_pos - v_position.xyz;
  vec3 halfvector = (v + l) / dot(v + l, v + l);

  vec3 ambient = ka * Ia;
  vec3 diffuse = kd * (u_light_intensity / dot(l, l)) * max(0.0, dot(normalize(l), nd));
  vec3 specular = ks * (u_light_intensity / dot(l, l)) * pow(max(0.0, dot(normalize(halfvector), nd)), p);
  vec3 color = ambient + diffuse + specular;

  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}
