#include "polymesh.h"
#include "linedrawer.h"
#include "framebuffer.h"
#include "camera.h"
#include "ray.h"
#include "light.h"
#include "vertex.h"
#include "vector.h"
#include "sphere.h"
#include "hit.h"
#include "object.h"
#include "photon.h"

#include "nanoflann.hpp"
#include "utils.h"

#include <float.h>
#include <iostream>
#include <cmath>
#include <time.h>
#include <cstdlib>

typedef float num_t;

int main(int argc, char *argv[]) {
	FrameBuffer *fb = new FrameBuffer(1024, 1024);
	Transform *transform = new Transform(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -2.0f, 0.0f, 1.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 1.0f);
	PolyMesh *pm = new PolyMesh((char *)"teapot.ply", transform, 0.3f, 0.3f, 0.3f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
	Camera *cam = new Camera(new Vertex(0.0f,2.5f,-5.0f), new Vector(0.0f,0.0f,1.0f), new Vector(0.0f,1.0f,0.0f), 1000.0f); //create camera
	Light *light = new Light(new Vector(0, 4.9999, 2.5), 1, 1, 1, 1); //create light

	PolyMesh floor((char *)"floor.ply", 0.0f, 0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
	PolyMesh ceiling((char *)"ceiling.ply", 0.0f, 0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
	PolyMesh farwall((char *)"farwall.ply", 0.0f, 0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
	PolyMesh leftwall((char *)"leftwall.ply", 0.0f, 0.9f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
	PolyMesh rightwall((char *)"rightwall.ply", 0.0f, 0.9f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

	Vector dir;
	Vector tempVector;
	Vector tempVector2;
	float totalr;
	float totalg;
	float totalb;
	float tempFloat;

	float t = FLT_MAX;
	float t_to_light;

	Vertex sphere1center(-1, 0.75, 1); 
	Sphere sphere1(sphere1center, 0.75, 0.0f, 0.0f, 0.0f, 0.1f, 0.9f, 1.0f, 1.0f, 1.0f); //create sphere1

	Vertex sphere2center(1, 1, 3.5);
	Sphere sphere2(sphere2center, 1, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f); //create sphere2

	Vector intersectToLight;
	Vertex hitPlusABit;
	
	Object* scene[] = { &floor, &ceiling, &farwall, &leftwall , &rightwall, &sphere1, &sphere2 }; //an array of objects that include all objects in the scene
	
	PointCloud<float> photonMap;  //most code from nanoflann to create kd tree
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud<float> >, PointCloud<float>, 3> KDTree;
	dump_mem_usage();
	KDTree photonMapKD(3, photonMap, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	const size_t numberOfPhotons = 10000000; //this is number of photon hits instead of photons being sent from light
	photonMap.pts.resize(numberOfPhotons);
	photonMap.col.resize(numberOfPhotons);
	photonMap.pow.resize(numberOfPhotons);
	int index = 0;
	srand(time(NULL));

	while (index < numberOfPhotons) {
		float x = 2;
		float y = 2;
		float z = 2;
		while (x * x + y * y + z * z > 1 || y > 0) {			//generates uniform random direction within a sphere, can be directed depending on light direction, e.g y > 0 means photon must be point downwards
			x = 2 * float(rand()) / float(RAND_MAX) - 1;
			y = 2 * float(rand()) / float(RAND_MAX) - 1;
			z = 2 * float(rand()) / float(RAND_MAX) - 1;
		}

		Vector photonDir(x, y, z);			//variables for intersection test
		photonDir.normalise();
		Vertex lightPos(light->pos.x, light->pos.y, light->pos.z);
		Ray photonRay(lightPos, photonDir);
		t = FLT_MAX;
		Hit photonClosestHit;
		Ray tempRay;
		int bounces = 0;
		bool absorbFlag = false;
		Object *previousObject;
		float cosineWeight;	

		while (absorbFlag == false && index < numberOfPhotons) { //if photon is not absorbed and photon map is not full
			bool hitFlag = false;
			for (Object *object : scene) {			//intersection test with every object in scene
				Hit photonHit;
				object->intersection(photonRay, photonHit);
				if (photonHit.flag == true) {
					if (photonHit.t < t) {
						photonClosestHit.position = photonHit.position;			//records information on closest obejct
						photonClosestHit.what = photonHit.what;
						photonClosestHit.flag = true;
						photonClosestHit.t = photonHit.t;
						photonClosestHit.normal = photonHit.normal;
						t = photonHit.t;
						hitFlag = true;
					}
				}
			}
			if (hitFlag == true) {
				bounces++;

				if (bounces == 1) {										
					previousObject = photonClosestHit.what;
				}

				float r = float(rand()) / float(RAND_MAX);			//random number from 0-1 for russian roulette 

				if (r < photonClosestHit.what->diffuseStrength) {  //photon is diffusely reflected from russian roulette
					if (bounces == 1) {								//if first bounce, do not store photon
						x = 2 * float(rand()) / float(RAND_MAX) - 1;  
						y = 2 * float(rand()) / float(RAND_MAX) - 1;
						z = 2 * float(rand()) / float(RAND_MAX) - 1;
						while (x * x + y * y + z * z > 1 || (x * photonClosestHit.normal.x + y * photonClosestHit.normal.y + z * photonClosestHit.normal.z) < 0) {
							x = 2 * float(rand()) / float(RAND_MAX) - 1;
							y = 2 * float(rand()) / float(RAND_MAX) - 1;
							z = 2 * float(rand()) / float(RAND_MAX) - 1;
						}
						photonDir.x = x;
						photonDir.y = y;
						photonDir.z = z;
						photonDir.normalise();

						cosineWeight = photonDir.dot(photonClosestHit.normal);

						lightPos.x = photonClosestHit.position.x + photonDir.x * 0.0001;
						lightPos.y = photonClosestHit.position.y + photonDir.y * 0.0001;
						lightPos.z = photonClosestHit.position.z + photonDir.z * 0.0001;

						photonRay.position = lightPos;
						photonRay.direction = photonDir;
					}

					else if (bounces == 2) {			//if second bounce store photon, seperated from bounce > 2 because colour is calculated from light colour here instead of previous photon colour, see arrows below
						photonMap.pts[index].x = photonClosestHit.position.x;
						photonMap.pts[index].y = photonClosestHit.position.y;
						photonMap.pts[index].z = photonClosestHit.position.z;

						photonMap.col[index].r = previousObject->red * light->red; // <----------- from light colour
						photonMap.col[index].g = previousObject->green * light->green; // <----------- from light colour
						photonMap.col[index].b = previousObject->blue * light->blue; // <----------- from light colour

						photonMap.pow[index].power = cosineWeight;

						index++;

						x = 2 * float(rand()) / float(RAND_MAX) - 1;
						y = 2 * float(rand()) / float(RAND_MAX) - 1;
						z = 2 * float(rand()) / float(RAND_MAX) - 1;
						while (x * x + y * y + z * z > 1 || (x * photonClosestHit.normal.x + y * photonClosestHit.normal.y + z * photonClosestHit.normal.z) < 0) {
							x = 2 * float(rand()) / float(RAND_MAX) - 1;
							y = 2 * float(rand()) / float(RAND_MAX) - 1;
							z = 2 * float(rand()) / float(RAND_MAX) - 1;
						}

						photonDir.x = x;
						photonDir.y = y;
						photonDir.z = z;
						photonDir.normalise();

						cosineWeight = photonDir.dot(photonClosestHit.normal);

						lightPos.x = photonClosestHit.position.x + photonDir.x * 0.0001;
						lightPos.y = photonClosestHit.position.y + photonDir.y * 0.0001;
						lightPos.z = photonClosestHit.position.z + photonDir.z * 0.0001;

						photonRay.position = lightPos;
						photonRay.direction = photonDir;
					}

					else if (bounces > 2) {				//seperated from bounce == 2 because colour is calculated from previous colour instead of light colour here, see arrows below
						if ((previousObject->red * photonMap.col[index - 1].r) > 0 || (previousObject->green * photonMap.col[index - 1].g) > 0 || (previousObject->blue * photonMap.col[index - 1].b) > 0) { //if light intensity is not 0 at r,g and b
							photonMap.pts[index].x = photonClosestHit.position.x;
							photonMap.pts[index].y = photonClosestHit.position.y;
							photonMap.pts[index].z = photonClosestHit.position.z;

							photonMap.col[index].r = (previousObject->red * photonMap.col[index - 1].r);	// <----------- from previous photon
							photonMap.col[index].g = (previousObject->green * photonMap.col[index - 1].g);	// <----------- from previous photon
							photonMap.col[index].b = (previousObject->blue * photonMap.col[index - 1].b);	// <----------- from previous photon

							photonMap.pow[index].power = cosineWeight * photonMap.pow[index - 1].power;

							index++;

							x = 2 * float(rand()) / float(RAND_MAX) - 1;
							y = 2 * float(rand()) / float(RAND_MAX) - 1;
							z = 2 * float(rand()) / float(RAND_MAX) - 1;
							while (x * x + y * y + z * z > 1 || (x * photonClosestHit.normal.x + y * photonClosestHit.normal.y + z * photonClosestHit.normal.z) < 0) {
								x = 2 * float(rand()) / float(RAND_MAX) - 1;
								y = 2 * float(rand()) / float(RAND_MAX) - 1;
								z = 2 * float(rand()) / float(RAND_MAX) - 1;
							}
							photonDir.x = x;
							photonDir.y = y;
							photonDir.z = z;
							photonDir.normalise();

							cosineWeight = photonDir.dot(photonClosestHit.normal);

							lightPos.x = photonClosestHit.position.x + photonDir.x * 0.0001;
							lightPos.y = photonClosestHit.position.y + photonDir.y * 0.0001;
							lightPos.z = photonClosestHit.position.z + photonDir.z * 0.0001;

							photonRay.position = lightPos;
							photonRay.direction = photonDir;
						}

						else { //if light intensity is 0 at r,g and b considere absorbed
							absorbFlag = true;
						}
					}
				}

				else if (r < (photonClosestHit.what->specularStrength + photonClosestHit.what->diffuseStrength + photonClosestHit.what->reflectedStrength)) { //if it hits specular object simply reflect photon and dont store photon on that hit
					tempRay = photonRay.reflect(photonClosestHit);
					photonRay = tempRay;
				}

				else { //if absorbed
					if (bounces == 2) {		//stores photon if bounces >= 2, same reason as above for seperating ==2 and > 2
						photonMap.pts[index].x = photonClosestHit.position.x;
						photonMap.pts[index].y = photonClosestHit.position.y;
						photonMap.pts[index].z = photonClosestHit.position.z;

						photonMap.col[index].r = previousObject->red * light->red;
						photonMap.col[index].g = previousObject->green * light->green;
						photonMap.col[index].b = previousObject->blue * light->blue;

						photonMap.pow[index].power = cosineWeight;

						index++;
					}

					else if (bounces > 2) {
						if ((previousObject->red * photonMap.col[index - 1].r) > 0 || (previousObject->green * photonMap.col[index - 1].g) > 0 || (previousObject->blue * photonMap.col[index - 1].b) > 0) {
							photonMap.pts[index].x = photonClosestHit.position.x;
							photonMap.pts[index].y = photonClosestHit.position.y;
							photonMap.pts[index].z = photonClosestHit.position.z;

							photonMap.col[index].r = (previousObject->red * photonMap.col[index - 1].r);
							photonMap.col[index].g = (previousObject->green * photonMap.col[index - 1].g);
							photonMap.col[index].b = (previousObject->blue * photonMap.col[index - 1].b);

							photonMap.pow[index].power = cosineWeight * photonMap.pow[index-1].power;

							index++;
						}
					}
					absorbFlag = true;
				}
			}

			else {
				absorbFlag = true;
			}
		}
	}

	size_t chunk_size = 100;	//code from nanoflann, inserts 100 points at once
	for (size_t i = 0; i < numberOfPhotons; i = i + chunk_size)
	{
		size_t end = std::min(size_t(i + chunk_size), numberOfPhotons - 1);
		// Inserts all points from [i, end]
		photonMapKD.addPoints(i, end);
	}
	




















	PointCloud<float> photonMapHD;		//photon map for caustics
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud<float> >, PointCloud<float>, 3> KDTree;
	dump_mem_usage();
	KDTree photonMapKDHD(3, photonMapHD, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	const size_t numberOfPhotonsHD = 1000000;
	photonMapHD.pts.resize(numberOfPhotonsHD);
	photonMapHD.col.resize(numberOfPhotonsHD);
	photonMapHD.pow.resize(numberOfPhotonsHD);
	int indexHD = 0;
	srand(time(NULL));

	while (indexHD < numberOfPhotonsHD) {
		float x = 2;
		float y = 2;
		float z = 2;
		while (x * x + y * y + z * z > 1 || y > 0 || z > 0 || x > 0) {		//direction of photons is manually directed towards glass sphere
			x = 2 * float(rand()) / float(RAND_MAX) - 1;
			y = 2 * float(rand()) / float(RAND_MAX) - 1;
			z = 2 * float(rand()) / float(RAND_MAX) - 1;
		}

		Vector photonDirHD(x, y, z);
		photonDirHD.normalise();
		Vertex lightPosHD(light->pos.x, light->pos.y, light->pos.z);
		Ray photonRayHD(lightPosHD, photonDirHD);
		t = 100;
		Hit photonClosestHitHD;
		Ray tempRayHD;
		bool absorbFlagHD = false;
		Object *previousObjectHD;

		bool hitFlagHD = false;
		for (Object *object : scene) {
			Hit photonHitHD;
			object->intersection(photonRayHD, photonHitHD);
			if (photonHitHD.flag == true) {
				if (photonHitHD.t < t) {
					photonClosestHitHD.position = photonHitHD.position;
					photonClosestHitHD.what = photonHitHD.what;
					photonClosestHitHD.flag = true;
					photonClosestHitHD.t = photonHitHD.t;
					photonClosestHitHD.normal = photonHitHD.normal;
					t = photonHitHD.t;
					hitFlagHD = true;
				}
			}
		}
		if (hitFlagHD == true) {
			if (photonClosestHitHD.what->refractedStrength > 0) { //checks if object hit is refractive since this is only for caustics
				Ray refractedRay;																//refraction calculations to get the ray that leaves the object
				refractedRay = photonRayHD.refract(photonClosestHitHD, 1.003, 1.52);
				Hit closestRefractedHit;
				Hit exitHit;
				photonClosestHitHD.what->intersection(refractedRay, exitHit);
				exitHit.normal.negate();
				Ray refractedExitRay;
				refractedExitRay = refractedRay.refract(exitHit, 1.52, 1.003);
				t = 100;
				hitFlagHD = false;

				for (Object *objectRefracted : scene) {				//exit ray intersection with scene
					Hit refractedHit;
					objectRefracted->intersection(refractedExitRay, refractedHit);
					if (refractedHit.flag == true && refractedHit.t < t) {
						hitFlagHD = true;
						t = refractedHit.t;
						closestRefractedHit.position = refractedHit.position;
						closestRefractedHit.what = refractedHit.what;
						closestRefractedHit.flag = true;
						closestRefractedHit.t = refractedHit.t;
						closestRefractedHit.normal = refractedHit.normal;
					}
				}

				if (hitFlagHD == true) {		//store photon information if exit ray hits something
					photonMapHD.pts[indexHD].x = closestRefractedHit.position.x;
					photonMapHD.pts[indexHD].y = closestRefractedHit.position.y;
					photonMapHD.pts[indexHD].z = closestRefractedHit.position.z;

					photonMapHD.col[indexHD].r = light->red;
					photonMapHD.col[indexHD].g = light->green;
					photonMapHD.col[indexHD].b = light->blue;

					photonMapHD.pow[indexHD].power = light->strength;

					indexHD++;
				}
			}
		}
	}

	size_t chunk_sizeHD = 100;
	for (size_t i = 0; i < numberOfPhotonsHD; i = i + chunk_sizeHD)
	{
		size_t end = std::min(size_t(i + chunk_sizeHD), numberOfPhotonsHD - 1);
		// Inserts all points from [i, end]
		photonMapKDHD.addPoints(i, end);
	}
























	for (int i = 0; i < 1024; i++)	//image of 1024 x 1024
	{
		for (int j = 0; j < 1024; j++)
		{
			cam->u.scalarMulti(((float)(i)-512), dir);
			cam->v.scalarMulti(((float)(j)-512), tempVector);
			dir.plus(tempVector, tempVector2);
			cam->w.scalarMulti(cam->dist, tempVector);
			tempVector2.minus(tempVector, dir);
			dir.normalise();
			Ray ray(cam->eye, dir);
			t = FLT_MAX;
			bool hitFlag = false;
			Hit closestHit;
			bool reflectedFlag = false;
			Hit closestReflectedHit;
			bool refractedFlag = false;
			Hit closestRefractedHit;
			int depth = 0;

			for (Object *object : scene) {  //intersection test with scene
				Hit hit;
				object->intersection(ray, hit);
				if (hit.flag == true) {
					if (hit.t < t) {
						t = hit.t;
						closestHit.position = hit.position;
						closestHit.what = hit.what;
						closestHit.flag = true;
						closestHit.t = hit.t;
						closestHit.normal = hit.normal;
						hitFlag = true;
					}
				}
			}

			if (hitFlag == true) {		//if ray hits an object
				intersectToLight.x = light->pos.x - closestHit.position.x;		//setting variables for shadow test
				intersectToLight.y = light->pos.y - closestHit.position.y;
				intersectToLight.z = light->pos.z - closestHit.position.z;
				t_to_light = (float)sqrt((double)(intersectToLight.x*intersectToLight.x + intersectToLight.y * intersectToLight.y + intersectToLight.z * intersectToLight.z));
				intersectToLight.normalise();
				hitPlusABit.x = closestHit.position.x + intersectToLight.x*0.0001;
				hitPlusABit.y = closestHit.position.y + intersectToLight.y*0.0001;
				hitPlusABit.z = closestHit.position.z + intersectToLight.z*0.0001;
				Ray shadowRay(hitPlusABit, intersectToLight);
				Hit shadowHit;

				for (Object *objectShadow : scene) {
					objectShadow->intersection(shadowRay, shadowHit);

					if (shadowHit.flag == true && shadowHit.t < t_to_light) {
						ray.shadowFlag = true;
					}
				}


				totalr = closestHit.what->red * ray.localLighting(closestHit.what->ambientStrength, closestHit.what->diffuseStrength, closestHit.what->specularStrength, light->strength, light->pos, closestHit, cam->eye);  //lighting calculation using phong
				totalg = closestHit.what->green * ray.localLighting(closestHit.what->ambientStrength, closestHit.what->diffuseStrength, closestHit.what->specularStrength, light->strength, light->pos, closestHit, cam->eye);
				totalb = closestHit.what->blue * ray.localLighting(closestHit.what->ambientStrength, closestHit.what->diffuseStrength, closestHit.what->specularStrength, light->strength, light->pos, closestHit, cam->eye);

				if (hitFlag == true && closestHit.what->reflectedStrength > 0) {	//if object is specular
					depth++;
					Ray reflectedRay;
					reflectedRay = ray.reflect(closestHit);			//generate reflected ray
					t = 100;
					hitFlag = false;

					for (Object *objectReflected : scene) {		//reflected ray intersection test
						Hit reflectedHit;
						objectReflected->intersection(reflectedRay, reflectedHit);
						if (reflectedHit.flag == true && reflectedHit.t < t) {
							hitFlag = true;
							t = reflectedHit.t;
							closestReflectedHit.position = reflectedHit.position;
							closestReflectedHit.what = reflectedHit.what;
							closestReflectedHit.flag = true;
							closestReflectedHit.t = reflectedHit.t;
							closestReflectedHit.normal = reflectedHit.normal;
						}
					}
						

					if (closestHit.what->refractedStrength == 0) { //if object is only reflective and not refractive (see through)
						reflectedFlag = true;
						intersectToLight.x = light->pos.x - closestHit.position.x;
						intersectToLight.y = light->pos.y - closestHit.position.y;
						intersectToLight.z = light->pos.z - closestHit.position.z;
						intersectToLight.normalise();	
						hitPlusABit.x = closestHit.position.x + intersectToLight.x*0.0001;
						hitPlusABit.y = closestHit.position.y + intersectToLight.y*0.0001;
						hitPlusABit.z = closestHit.position.z + intersectToLight.z*0.0001;
						Ray reflectShadowRay(hitPlusABit, intersectToLight);
						Hit reflectShadowHit;
						Vector reflectedEye;

						for (Object *objectShadow : scene) {
							objectShadow->intersection(reflectShadowRay, reflectShadowHit);

							if (shadowHit.flag == true) {
								reflectedRay.shadowFlag = true;
							}
						}

						totalr = totalr + closestReflectedHit.what->red * closestHit.what->reflectedStrength * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);
						totalg = totalg + closestReflectedHit.what->green * closestHit.what->reflectedStrength * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);
						totalb = totalb + closestReflectedHit.what->blue * closestHit.what->reflectedStrength * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);
					}

					if (closestHit.what->refractedStrength > 0) {		//if object is refractive
						Ray refractedRay;
						refractedRay = ray.refract(closestHit, 1.003, 1.52);
						Hit exitHit;
						closestHit.what->intersection(refractedRay, exitHit);
						exitHit.normal.negate();
						Ray refractedExitRay;
						refractedExitRay = refractedRay.refract(exitHit, 1.52, 1.003);
						t = 100;
						hitFlag = false;

						float eta = 1.52 / 1.003;		//fresnel calculations
						float rpar = (eta * ray.viewer_dir.dot(closestHit.normal) + refractedRay.direction.dot(closestHit.normal)) / (eta * ray.viewer_dir.dot(closestHit.normal) - refractedRay.direction.dot(closestHit.normal));
						float rper = (ray.viewer_dir.dot(closestHit.normal) + eta * refractedRay.direction.dot(closestHit.normal)) / (ray.viewer_dir.dot(closestHit.normal) - eta * refractedRay.direction.dot(closestHit.normal));
						float kr = 0.5 * (rpar * rpar + rper * rper);
						float kt = 1 - kr;

						totalr = totalr + closestReflectedHit.what->red * kr * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);
						totalg = totalg + closestReflectedHit.what->green * kr * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);
						totalb = totalb + closestReflectedHit.what->blue * kr * reflectedRay.localLighting(closestReflectedHit.what->ambientStrength, closestReflectedHit.what->diffuseStrength, closestReflectedHit.what->specularStrength, light->strength, light->pos, closestReflectedHit, closestHit.position);

						for (Object *objectRefracted : scene) {		//refractive ray intersection test
							Hit refractedHit;
							objectRefracted->intersection(refractedExitRay, refractedHit);
							if (refractedHit.flag == true && refractedHit.t < t) {
								hitFlag = true;
								t = refractedHit.t;
								closestRefractedHit.position = refractedHit.position;
								closestRefractedHit.what = refractedHit.what;
								closestRefractedHit.flag = true;
								closestRefractedHit.t = refractedHit.t;
								closestRefractedHit.normal = refractedHit.normal;
							}
						}

						if (hitFlag == true) {
							refractedFlag = true;
							totalr = totalr + closestRefractedHit.what->red * kt * refractedExitRay.localLighting(closestRefractedHit.what->ambientStrength, closestRefractedHit.what->diffuseStrength, closestRefractedHit.what->specularStrength, light->strength, light->pos, closestRefractedHit, exitHit.position);
							totalg = totalg + closestRefractedHit.what->green * kt * refractedExitRay.localLighting(closestRefractedHit.what->ambientStrength, closestRefractedHit.what->diffuseStrength, closestRefractedHit.what->specularStrength, light->strength, light->pos, closestRefractedHit, exitHit.position);
							totalb = totalb + closestRefractedHit.what->blue * kt * refractedExitRay.localLighting(closestRefractedHit.what->ambientStrength, closestRefractedHit.what->diffuseStrength, closestRefractedHit.what->specularStrength, light->strength, light->pos, closestRefractedHit, exitHit.position);
						}
					}
				}

				float query_pt[3];	//point at which to get indirect illumination from in photon map
				if (reflectedFlag == true) { //if ray has been reflected set to end of reflected ray
					query_pt[0] = closestReflectedHit.position.x;
					query_pt[1] = closestReflectedHit.position.y;
					query_pt[2] = closestReflectedHit.position.z;
				}
				else if (refractedFlag == true) { //if ray has been refracted set to end of refracted ray
					query_pt[0] = closestRefractedHit.position.x;
					query_pt[1] = closestRefractedHit.position.y;
					query_pt[2] = closestRefractedHit.position.z;
				}
				else { //if ray is original ray, set to original intersect
					query_pt[0] = ray.position.x + ray.direction.x *t;
					query_pt[1] = ray.position.y + ray.direction.y *t;
					query_pt[2] = ray.position.z + ray.direction.z *t;
				}

				const size_t num_results = 200; //k of kNN search, this is for global photon map
				size_t ret_index[num_results];	//array of indexs of the photons in photon map
				num_t out_dist_sqr[num_results];	//distance squared of each corresponding photon
				nanoflann::KNNResultSet<num_t> resultSet(num_results);
				resultSet.init(ret_index, out_dist_sqr);
				photonMapKD.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

				float pi = 3.141592653589793284;
				float max_radius_sq = 0;
				for (float val : out_dist_sqr) {	//finds smallest radius which enclose all photons
					if (val > max_radius_sq) {
						max_radius_sq = val;
					}
				}

				float redAvg = 0;
				float greenAvg = 0;
				float blueAvg = 0;
				for (size_t index : ret_index) {
					redAvg = redAvg + (photonMap.col[index].r * photonMap.pow[index].power) / num_results;		//colour average of all photons in radius
					greenAvg = greenAvg + (photonMap.col[index].g * photonMap.pow[index].power) / num_results;
					blueAvg = blueAvg + (photonMap.col[index].b * photonMap.pow[index].power) / num_results;
				}

				const size_t num_resultsHD = 50;	//same code for caustic photon map
				size_t ret_indexHD[num_resultsHD];
				num_t out_dist_sqrHD[num_resultsHD];
				nanoflann::KNNResultSet<num_t> resultSetHD(num_resultsHD);
				resultSetHD.init(ret_indexHD, out_dist_sqrHD);
				photonMapKDHD.findNeighbors(resultSetHD, query_pt, nanoflann::SearchParams(10));

				float max_radius_sqHD = 0;
				for (float val : out_dist_sqrHD) {
					if (val > max_radius_sqHD) {
						max_radius_sqHD = val;
					}
				}

				float redAvgHD = 0;
				float greenAvgHD = 0;
				float blueAvgHD = 0;
				for (size_t index : ret_indexHD) {
					redAvgHD = redAvgHD + (photonMapHD.col[index].r * photonMapHD.pow[index].power) / num_resultsHD;
					greenAvgHD = greenAvgHD + (photonMapHD.col[index].g * photonMapHD.pow[index].power) / num_resultsHD;
					blueAvgHD = blueAvgHD + (photonMapHD.col[index].b * photonMapHD.pow[index].power) / num_resultsHD;
				}

				//totalr = redAvgHD * (1 / (pi*max_radius_sqHD)) / 10000000;
				//totalg = greenAvgHD * (1 / (pi*max_radius_sqHD)) / 10000000;
				//totalb = blueAvgHD * (1 / (pi*max_radius_sqHD)) / 10000000;

				//totalr = redAvg;
				//totalg = greenAvg;
				//totalb = blueAvg;
				
				//totalr = 0.8 * totalr + 0.2 * redAvg;
				//totalg = 0.8 * totalg + 0.2 * greenAvg;
				//totalb = 0.8 * totalb + 0.2 * blueAvg;

				totalr = totalr + redAvgHD * (1 / (pi*max_radius_sqHD)) / 1000000 + redAvg * (1 / (pi*max_radius_sq)) / 2000;			//conbination of direct, indirect, caustic
				totalg = totalg + greenAvgHD * (1 / (pi*max_radius_sqHD)) / 1000000 + greenAvg * (1 / (pi*max_radius_sq)) / 2000;
				totalb = totalb + blueAvgHD * (1 / (pi*max_radius_sqHD)) / 1000000 + blueAvg * (1 / (pi*max_radius_sq)) / 2000;

				fb->plotPixel(1024 - i, 1024 - j, totalr, totalg, totalb);
			}
		}
	}
	fb->writeRGBFile((char *)"testRGB.ppm");
	return 0;
}