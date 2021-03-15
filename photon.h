#pragma once

#include "vertex.h"

struct Photon {
	Vertex position;
	Vector direction;
	float intensity;
	float flag;
};