#pragma once

#include "vertex.h"
#include "vector.h"

class Object;
class Sphere;

class Hit {
public:
	bool flag = false;
	float t;
	Object *what;
	Vertex position;
	Vector normal;
};
