#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform mat4 mvp_matrix;

attribute vec4 a_position;

void main()
{
    // Calculate vertex position in screen space
    gl_Position = mvp_matrix * a_position;

}
