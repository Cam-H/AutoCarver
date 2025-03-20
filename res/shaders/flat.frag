#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform vec3 out_color;

varying vec3 normal;

void main()
{
    float intensity = (dot(normal, vec3(0.0, 1.0, 0.0)) + 1.0) * 0.5;
    gl_FragColor = vec4(0.1 * out_color + 0.9 * out_color * intensity, 1.0);
}

