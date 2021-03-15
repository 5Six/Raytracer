#pragma once

#include "vertex.h"
#include "object.h"

class Sphere : public Object {
	Vertex center;
	float  radius;

public:
	Sphere(Vertex c, float r, float a, float d, float s, float reflect, float refract, float r2, float g, float b);
	void intersection(Ray ray, Hit &hit);
	void shadowTest(Ray ray, Hit &hit);
};
