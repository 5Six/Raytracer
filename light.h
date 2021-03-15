#pragma once
#include "vector.h"
class Light {
public:
	Vector pos;
	float strength;
	float red;
	float green;
	float blue;

	Light(Vector *p, float str, float r, float g, float b)
	{
		pos.x = p->x;
		pos.y = p->y;
		pos.z = p->z;
		strength = str;
		red = r;
		blue = b;
		green = g;
	}
};