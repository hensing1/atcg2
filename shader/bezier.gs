#version 330 core

layout (points) in;
layout (line_strip, max_vertices=2) out;

in VS_OUT 
{
	int instance;
    vec3 points[4];
}gs_in[];

vec3 bezier(float u, vec3 p0, vec3 p1, vec3 p2, vec3 p3)
{
    vec3 p;
    //
    return p;
}

uniform int discretization;
uniform mat4 P;

void main()
{
    vec3 p0 = gs_in[0].points[0];
    vec3 p1 = gs_in[0].points[1];
    vec3 p2 = gs_in[0].points[2];
    vec3 p3 = gs_in[0].points[3];
    gl_Position = P * vec4(bezier(gs_in[0].instance/float(discretization), p0, p1, p2, p3),1);
    EmitVertex();
    gl_Position = P * vec4(bezier((gs_in[0].instance+1)/float(discretization), p0, p1, p2, p3),1);
    EmitVertex();
    EndPrimitive();
    
}