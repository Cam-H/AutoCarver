#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

varying vec3 normal;
varying vec3 color;

void main()
{
    float intensity = (dot(normal, vec3(0.0, 1.0, 0.0)) + 1.0) * 0.5;
    gl_FragColor = vec4(color * intensity, 1.0);
}

