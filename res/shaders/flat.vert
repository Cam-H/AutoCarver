#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;
uniform mat3 n_matrix;

attribute vec4 a_position;
attribute vec3 a_normal;

varying vec3 normal;

void main()
{
    // Calculate vertex position in screen space
    gl_Position = mvp_matrix * a_position;

    normal = n_matrix * a_normal;

}
