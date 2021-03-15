#pragma once

#include "vertex.h"
#include "vector.h"
#include "hit.h"

#include <float.h>
#include <cmath>
#include <iostream>

class Ray {
public:
	Vertex position;
	Vector direction;

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
	Vertex intersectVertex;
	Vector light_dir;
	Vector shadow_dir;
	Vector incident_dir;
	Vector reflected;
	Vector viewer_dir;
	Vector nearestTriangle;
	Vertex nearestTriangleVertex;
	Vector hit_n;
	Vector sphereIntersectPoint;

	float d;
	float t;
	float new_t = FLT_MAX;
	float t_to_light;

	float sphere_a;
	float sphere_b;
	float sphere_c;
	float sphere_t0;
	float sphere_t1;
	float discriminant;
	float tempFloat;

	Vertex hitPlusABit;

	bool shadowFlag = false;

	Ray()
	{
	}

	Ray(Vertex p, Vector d)
	{
		position = p;
		direction = d;
		position.toVector(pos);
	}

	/*
	bool intersectTriangle(Vertex vertex1, Vertex vertex2, Vertex vertex3) {
		vertex1.toVector(a);
		vertex2.toVector(b);
		vertex3.toVector(c);

		c.minus(a, n);
		b.minus(a, tempVector);

		n.cross(tempVector, tempVector2);
		tempVector2.normalise();
		tempVector2.copy(n);

		d = -(n.x * a.x + n.y * a.y + n.z * a.z);

		if (n.dot(direction) == 0.0f) {
			return false;
		}
		t = -(n.dot(pos) + d) / (n.dot(direction));
		if (t < 0) {
			return false;
		}
		direction.scalarMulti(t, intersectPoint);

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

		if (cross1.dot(cross2) > 0.0f && cross2.dot(cross3) > 0.0f && t < new_t) {
			new_t = t;
			intersectPoint.copy(nearestTriangle);
			n.copy(hit_n);
			return true;
		}
		else {
			return false;
		}
	}

	bool shadowTest(Vertex vertex1, Vertex vertex2, Vertex vertex3, Vector light_pos, Hit hit) {
		light_dir.x = hit.position.x - light_pos.x;
		light_dir.y = hit.position.y - light_pos.y;
		light_dir.z = hit.position.z - light_pos.z;
		t_to_light = (float)sqrt((double)(light_dir.x*light_dir.x + light_dir.y * light_dir.y + light_dir.z * light_dir.z));
		light_dir.normalise();
		light_dir.negate();
		nearestTriangle.x = hit.position.x + (light_dir.x*0.00001);
		nearestTriangle.y = hit.position.y + (light_dir.y*0.00001);
		nearestTriangle.z = hit.position.z + (light_dir.z*0.00001);
		nearestTriangleVertex.x = nearestTriangle.x;
		nearestTriangleVertex.y = nearestTriangle.y;
		nearestTriangleVertex.z = nearestTriangle.z;
		Ray shadowRay(nearestTriangleVertex, light_dir);
		if (shadowRay.intersectTriangle(vertex1, vertex2, vertex3) == true) {
			shadowFlag = true;
			return true;
		}
		else {
			return false;
		}
	}
	*/

	float localLighting(float aStr, float dStr, float sStr, float lightStr, Vector light_pos, Hit hit, Vertex eye) {
		float ambientContribution = 0;
		float diffuseContribution = 0;
		float specularContribution = 0;

		light_dir.x = hit.position.x - light_pos.x;
		light_dir.y = hit.position.y - light_pos.y;
		light_dir.z = hit.position.z - light_pos.z;

		light_dir.normalise();
		light_dir.copy(incident_dir);
		light_dir.negate();
		
		diffuseContribution = lightStr * dStr * (hit.normal.dot(light_dir));
		hit.normal.scalarMulti(2 * (incident_dir.dot(hit.normal)), tempVector);
		tempVector.minus(incident_dir, reflected);
		reflected.normalise();
		viewer_dir.x = -hit.position.x + eye.x;
		viewer_dir.y = -hit.position.y + eye.y;
		viewer_dir.z = -hit.position.z + eye.z;
		viewer_dir.normalise();

		light_dir.cross(hit.normal, tempVector);
		hit.normal.cross(viewer_dir, tempVector2);

		ambientContribution = aStr * lightStr;
		if (shadowFlag == true) {
			return ambientContribution;
		}

		if (hit.normal.dot(incident_dir) > 0 || acos(reflected.dot(viewer_dir) <= (acos(viewer_dir.dot(light_dir))))) {
			specularContribution = 0;
		}
		else {
			specularContribution = lightStr * sStr * pow(reflected.dot(viewer_dir), 10);
		}

		if (diffuseContribution > 0 && specularContribution > 0) {
			return (ambientContribution + diffuseContribution + specularContribution);
		}
		else if (diffuseContribution > 0 && specularContribution <= 0) {
			return (ambientContribution + diffuseContribution);
		}
		else if (diffuseContribution <= 0 && specularContribution > 0) {
			return (ambientContribution + specularContribution);
		}
		else {
			return ambientContribution;
		}
	}

	Ray reflect(Hit hit) {
		Vector reflectedEye;

		tempVector.x = direction.x;
		tempVector.y = direction.y;
		tempVector.z = direction.z;
		tempFloat = 2 * tempVector.dot(hit.normal);
		hit.normal.scalarMulti(tempFloat, tempVector2);
		reflectedEye.x = direction.x - tempVector2.x;
		reflectedEye.y = direction.y - tempVector2.y;
		reflectedEye.z = direction.z - tempVector2.z;
		reflectedEye.normalise();
		hitPlusABit.x = hit.position.x + reflectedEye.x*0.001;
		hitPlusABit.y = hit.position.y + reflectedEye.y*0.001;
		hitPlusABit.z = hit.position.z + reflectedEye.z*0.001;
		Ray reflectedRay(hitPlusABit, reflectedEye);
		return reflectedRay;
	}

	Ray refract(Hit hit, float index1, float index2) {
		float c1 = -hit.normal.dot(direction);
		float n = index1 / index2;
		float c2 = sqrt(1 - n*n * (1 - c1*c1));
		(direction.scalarMulti(n, tempVector));
		hit.normal.scalarMulti((n * c1 - c2), tempVector2);
		Vector Rr;
		tempVector.plus(tempVector2, Rr);
		Rr.normalise();

		Ray refractedRay;
		refractedRay.position.x = hit.position.x + Rr.x * 0.001;
		refractedRay.position.y = hit.position.y + Rr.y * 0.001;
		refractedRay.position.z = hit.position.z + Rr.z * 0.001;
		refractedRay.direction = Rr;

		return refractedRay;
	}
}; 
