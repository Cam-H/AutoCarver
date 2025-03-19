#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 u_transform;
uniform mat4 vp_matrix;

attribute vec4 a_position;
attribute vec3 a_normal;
attribute vec3 a_color;

varying vec3 normal;
varying vec3 color;

void main()
{
    // Calculate vertex position in screen space
    gl_Position = vp_matrix * u_transform * a_position;

    vec4 temp = u_transform * vec4(a_normal, 1.0);
    normal = vec3(temp.x, temp.y, temp.z);

    color = a_color;

}
