#pragma once

#include "vector.h"

class Vertex {
public:
	float x;
	float y;
	float z;
	float w;

	Vertex()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		w = 1.0f;
	}

	Vertex(float px, float py, float pz, float pw)
	{
		x = px;
		y = py;
		z = pz;
		w = pw;
	}

	Vertex(float px, float py, float pz)
	{
		x = px;
		y = py;
		z = pz;
		w = 1.0f;
	}

	void toVector(Vector &result) {
		result.x = x;
		result.y = y;
		result.z = z;
	}
};
