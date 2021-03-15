#pragma once

#include "vertex.h"
#include "transform.h"
#include "object.h"

typedef int TriangleIndex[3];

class PolyMesh : public Object{
public:
	int vertex_count;
	int triangle_count;
        Vertex *vertex;
	TriangleIndex *triangle;

	void do_construct(char *file, Transform *transform, float a, float d, float s, float reflect, float refract, float r, float g, float b);
	void intersection(Ray ray, Hit &hit);

	PolyMesh(char *file, float a, float d, float s, float reflect, float refract, float r, float g, float b);
	PolyMesh(char *file, Transform *transform, float a, float d, float s, float reflect, float refract, float r, float g, float b);
};
