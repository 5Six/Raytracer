#pragma once

#include "vertex.h"
#include <math.h>

class Vector {
public:
	float x;
	float y;
	float z;

	Vector(float px, float py, float pz)
	{
		x = px;
		y = py;
		z = pz;
	}

	Vector()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	void normalise()
	{
		float len = (float)sqrt((double)(x*x + y*y + z*z));
		x = x / len;
		y = y / len;
		z = z / len;
	}

	float dot(Vector &other)
	{
		return x*other.x + y*other.y + z*other.z;
	}


	void reflection(Vector initial, Vector &reflect)
	{
		float d;

		d = dot(initial);
		d = d * 2.0f;

		reflect.x = initial.x - d * x;
		reflect.y = initial.y - d * y;
		reflect.z = initial.z - d * z;
	}

	void negate()
	{
		x = -x;
		y = -y;
		z = -z;
	}

	void negateAlt(Vector &result) {
		result.x = -x;
		result.y = -y;
		result.z = -z;
	}

	void cross(Vector &other, Vector &result)
	{
	  result.x = y*other.z - z*other.y;
	  result.y = z*other.x - x*other.z;
	  result.z = x*other.y - y*other.x;
	}

	void plus(Vector &other, Vector &result) {
		result.x = x + other.x;
		result.y = y + other.y;
		result.z = z + other.z;
	}

	void minus(Vector &other, Vector &result) {
		result.x = x - other.x;
		result.y = y - other.y;
		result.z = z - other.z;
	}

	void scalarMulti(float scalar, Vector &result) {
		result.x = x * scalar;
		result.y = y * scalar;
		result.z = z * scalar;
	}

	void copy(Vector &result) {
		result.x = x;
		result.y = y;
		result.z = z;
	}

	 /*
	void toVertex(Vertex &result) {
		result.x = x;
		result.y = y;
		result.z = z;
	}
	*/
};
