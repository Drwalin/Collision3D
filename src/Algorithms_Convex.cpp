// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionAlgorithms.hpp"

namespace Collision3D
{
void TestPlane(glm::vec3 normal, float d, const RayInfo &ray, float &t,
			   bool &startsInFront, bool &isParallel)
{
	constexpr float EPS = 0.00000001f;

	const float a = glm::dot(ray.dir, normal);
	const float p = glm::dot(ray.start, normal);

	startsInFront = p > d + EPS;

	if (a <= -EPS && a >= EPS) {
		isParallel = false;
		// ray not parallel to plane
		t = (d - p) / a;
		if (a < 0.0f) {
			// ray points toward plane
		} else { // (a > 0.of)
				 // ray points outward plane
		}
	} else {
		isParallel = true;
	}
}

bool TestPlaneIterational(glm::vec3 normal, float d, const RayInfo &ray,
						  float &near, float &far, bool &useNormal)
{
	float t = +1e9;
	bool startsInFront = false;
	bool isParallel = false;
	TestPlane(normal, d, ray, t, startsInFront, isParallel);

	if (isParallel) {
		if (startsInFront) {
			// is parallel and starts outside of object
			return false; // case 5
		} else {
			// is parallel inside of object
			return true; // case 6
		}
	} else {
		if (startsInFront) {
			if (t < 0) {
				// face away from this plane being outside
				return false; // case 1
			} else {
				if (near < t) {
					near = t;
					useNormal = true;
				}
				return true; // case 2
			}
		} else {
			if (t < 0) {
				if (near < t) {
					near = t;
					useNormal = true;
				}
				return true; // case 4
			} else {
				// is inside, ignore
				far = std::min(far, t);
				return true; // case 3
			}
		}
	}
	return true;
}
} // namespace Collision3D

