#ifndef _OBJECT_H_
#define _OBJECT_H_

#include "ray.h"
#include "hit.h"

class Object {
public:

	Object *next;
	float ambientStrength;
	float diffuseStrength;
	float specularStrength;
	float reflectedStrength;
	float refractedStrength;
	float red;
	float green;
	float blue;

	Object()
	{
		next = (Object *)0;
	}
	
	virtual void intersection(Ray ray, Hit &hit)
	{

	}
};

#endif
