#include "vector.h"
#include "vertex.h"

class Camera {
public:
	Vertex eye;
	Vector look;
	Vector up;
	float dist;
	Vector w;
	Vector u;
	Vector v;

	Camera(Vertex *e, Vector *l, Vector *u2, float d)
	{
		eye.x = e->x;
		eye.y = e->y;
		eye.z = e->z;
		look.x = l->x;
		look.y = l->y;
		look.z = l->z;
		up.x = u2->x;
		up.y = u2->y;
		up.z = u2->z;
		dist = d;
		Vector eyeVector(eye.x, eye.y, eye.z);
		look.negateAlt(w);
		w.normalise();
		up.cross(w, u);
		u.normalise();
		w.cross(u, v);
		v.normalise();
	}
};