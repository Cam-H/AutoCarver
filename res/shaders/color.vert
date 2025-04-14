#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;
uniform mat3 n_matrix;

attribute vec4 a_position;
attribute vec3 a_normal;
attribute vec3 a_color;

varying vec3 normal;
varying vec3 color;

void main()
{
    // Calculate vertex position in screen space
    gl_Position = mvp_matrix * a_position;

    normal = n_matrix * a_normal;

    color = a_color;

}
