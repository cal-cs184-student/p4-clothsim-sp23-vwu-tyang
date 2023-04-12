#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

float ka = 0.1;
vec3 kd = u_color.xyz;
float ks = 0.5;
vec3 Ia = vec3(1.0);
float p = 100;

void main() {
  // YOUR CODE HERE
  vec3 v = u_cam_pos - v_position.xyz;
  vec3 l = u_light_pos - v_position.xyz;
  vec3 h = (v + l) / dot(v + l, v + l);
  
  // (Placeholder code. You will want to replace it.)
  vec3 ambient = ka * Ia;
  vec3 diffuse = kd * (u_light_intensity / dot(l, l)) * max(0, dot(normalize(l), normalize(v_normal.xyz)));
  vec3 specular = ks * (u_light_intensity / dot(l, l)) * pow(max(0, dot(normalize(h), normalize(v_normal.xyz))), p);
  out_color.xyz = ambient + diffuse + specular;
  out_color.a = 1;
}

