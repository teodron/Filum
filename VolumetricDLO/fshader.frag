uniform vec3 lightDir;
varying vec3 normal;
void main()
{
	float intensity = dot(lightDir, normal);
	vec4 col = gl_Color * intensity;
	//gl_FragColor =  0.5 * (1.5 + intensity) * gl_Color;
	intensity = 0.5 * (intensity + 1);
	gl_FragColor = col + intensity * vec4(1,1,0,1) + (1-intensity) * vec4(0.4,0.1,0.8,1);
}
