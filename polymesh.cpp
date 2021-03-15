#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "polymesh.h"

using namespace std;

PolyMesh::PolyMesh(char *file, float a, float d, float s, float reflect, float refract, float r, float g, float b)
{
  Transform *transform = new Transform();

  this->do_construct(file, transform, a, d, s, reflect, refract, r, g, b);
}

PolyMesh::PolyMesh(char *file, Transform *transform, float a, float d, float s, float reflect, float refract, float r, float g, float b)
{
	this->do_construct(file, transform, a, d, s, reflect, refract, r, g, b);
}

void PolyMesh::do_construct(char *file, Transform *transform, float a, float d, float s, float reflect, float refract, float r, float g, float b)
{
	ambientStrength = a;
	diffuseStrength = d;
	specularStrength = s;
	reflectedStrength = reflect;
	refractedStrength = refract;
	red = r;
	green = g;
	blue = b;

	ifstream myfile;
	myfile.open(file);

	string text;
	
	myfile >> text >> text >> text >> vertex_count;
	vertex = new Vertex[vertex_count];

	myfile >> text >> text >> triangle_count;
	triangle = new TriangleIndex[triangle_count];

	for (int i = 0; i < vertex_count; i += 1)
	{
		myfile >> vertex[i].x >> vertex[i].y >> vertex[i].z;
		transform->apply(vertex[i]);
	}

	for (int j = 0; j < triangle_count; j += 1)
	{
		myfile >> text >> triangle[j][0] >> triangle[j][1] >> triangle[j][2];
	}

	myfile.close();
}

void PolyMesh::intersection(Ray ray, Hit &hit) {
	Vector a;
	Vector b;
	Vector c;
	Vector n;
	Vector pos;
	Vector intersectPoint;
	Vector v1;
	Vector v2;
	Vector v3;
	Vector cross1;
	Vector cross2;
	Vector cross3;
	Vector tempVector;
	Vector tempVector2;
	Vector hit_n;

	float d;
	float t;
	float new_t = FLT_MAX;

	hit.flag = false;

	for (int k = 0; k < triangle_count; k += 1) {
		vertex[triangle[k][0]].toVector(a);
		vertex[triangle[k][1]].toVector(b);
		vertex[triangle[k][2]].toVector(c);

		c.minus(a, n);
		b.minus(a, tempVector);

		tempVector.cross(n, tempVector2);
		tempVector2.copy(n);

		pos.x = ray.position.x;
		pos.y = ray.position.y;
		pos.z = ray.position.z;

		n.normalise();

		d = -(n.x * a.x + n.y * a.y + n.z * a.z);

		if (n.dot(ray.direction) == 0.0f) {
			continue;
		}
		t = -(n.dot(pos) + d) / (n.dot(ray.direction));
		if (t < 0) {
			continue;
		}
		ray.direction.scalarMulti(t, intersectPoint);

		intersectPoint.plus(pos, tempVector);

		tempVector.copy(intersectPoint);

		b.minus(a, v1);
		c.minus(b, v2);
		a.minus(c, v3);

		intersectPoint.minus(a, tempVector);
		tempVector.cross(v1, cross1);
		intersectPoint.minus(b, tempVector);
		tempVector.cross(v2, cross2);
		intersectPoint.minus(c, tempVector);
		tempVector.cross(v3, cross3);

		if (cross1.dot(cross2) > -0.000001f && cross2.dot(cross3) > -0.000001f && cross1.dot(cross3) > -0.000001f && t < new_t) {
			new_t = t;
			n.copy(hit_n);
			hit.what = this;
			hit.normal = n;
			hit.position.x = intersectPoint.x;
			hit.position.y = intersectPoint.y;
			hit.position.z = intersectPoint.z;
			hit.t = t;
			hit.flag = true;
		}
	}
}
