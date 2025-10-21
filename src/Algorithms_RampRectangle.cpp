// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstdio>

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

extern Collision3D::RampRectangle RAMP_RECT_GLOB;
extern Collision3D::Transform RAMP_RECT_GLOB_TRANS;

Collision3D::RampRectangle RAMP_RECT_GLOB = {};
Collision3D::Transform RAMP_RECT_GLOB_TRANS = {};

extern bool DO_PRINT;
#define printf(...) {if(DO_PRINT){printf(__VA_ARGS__);}}

namespace Collision3D
{
using namespace spp;

spp::Aabb RampRectangle::GetAabb(const Transform &trans) const
{
	const float h = fabs(halfHeightSkewness) + halfThickness;
	Transform t = trans;
	t.pos.y -= h;
	return VertBox{{halfWidth, h, halfDepth}}.GetAabb(trans);
}

bool RampRectangle::RayTest(const Transform &trans, const RayInfo &ray,
							float &near, glm::vec3 &normal) const
{
	const RayInfo loc = trans.ToLocal(ray);
	printf("ray start = %7.2f %7.2f %7.2f\n", loc.start.x, loc.start.y, loc.start.z);
	if (RayTestLocal(trans.ToLocal(ray), near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool RampRectangle::RayTestLocal(const RayInfo &ray, float &near,
								 glm::vec3 &normal) const
{
	const glm::vec3 non{0, halfDepth, -halfHeightSkewness};
	const glm::vec3 no = non;
	const glm::vec3 n[6] = {{0, 0, -1}, {0, 0, 1}, {1, 0, 0},
							{-1, 0, 0}, no,		   -no};
	const float ofn = halfThickness * no.y;
	const float offs[6] = {halfDepth, halfDepth, halfWidth,
						   halfWidth, ofn,		 ofn};
	int frontNormalId = -1, backNormalId = -1;

	near = -1e9f;
	float far = 1e9f;

	for (int i = 0; i < 6; ++i) {
		if (TestPlaneIterational(n[i], offs[i], ray, near, far, frontNormalId,
								 backNormalId, i) == false) {
			printf("ret %i/6\n", i);
			return false;
		}
	}

	if (far < 0.0f) {
		printf("ret 1");
		return false;
	}

	if (far < near) {
		printf("ret 2");
		return false;
	}

	if (near < 0.0f) {
		/* is inside*/
		near = 0.0f;
		normal = -ray.dirNormalized;
		/* find shortest way outside */
		float d = glm::dot(ray.start, n[0]) - offs[0];
		assert(d <= 0.0f);
		for (int i = 1; i < 6; ++i) {
			const float d2 = glm::dot(ray.start, n[i]) - offs[i];
			assert(d2 <= 0.0f);
			if (d > d2) {
				d = d2;
				normal = n[i];
			}
		}
		return true;
	} else if (near >= 0.0f) {
		/* outside, hitting front face */
		if (near <= 1.0f) {
			normal = n[frontNormalId];
			printf("Colliding with normal: %5.2f %5.2f %5.2f\n", normal.x, normal.y, normal.z);
			return true; // frontface
		} else {
			printf("ret 3");
			return false;
		}
	} else {
		if (far < near) {
			/* inside, hitting back face */
			normal = n[backNormalId];
			near = 0.0f; // near = far;
			return true; // backface
		} else {
			/* inside, but back face beyond tmax */
			printf("ret 4");
			return false; // missed
		}
	}
}

bool RampRectangle::CylinderTestOnGround(const Transform &trans,
										 const Cylinder &cyl, glm::vec3 pos,
										 float &offsetHeight,
										 glm::vec3 *onGroundNormal,
										 bool *isOnEdge) const
{
	printf("onground pos = %7.2f %7.2f %7.2f\n", pos.x, pos.y, pos.z);
	
	if (fabs(halfHeightSkewness) > halfDepth) {
		return false;
	}

	const glm::vec3 localPos = trans.ToLocal(pos);

	if (fabs(localPos.x) > halfWidth + cyl.radius + ON_EDGE_FACTOR) {
		return false;
	}

	if (fabs(localPos.z) > halfDepth + ON_EDGE_FACTOR) {
		return false;
	}

	const float y =
		(halfHeightSkewness * localPos.z) / halfDepth + halfThickness;
	offsetHeight = localPos.y - y;

	if (onGroundNormal) {
		*onGroundNormal = glm::vec3(0, halfDepth, -halfHeightSkewness);
	}

	if (isOnEdge) {
		if (fabs(localPos.x) > halfWidth + cyl.radius) {
			*isOnEdge = true;
		} else if (fabs(localPos.z) > halfDepth) {
			*isOnEdge = true;
		}
	}
	
	printf("              offsetHeight = %.2f\n", offsetHeight);

	return true;
}

bool RampRectangle::CylinderTestMovement(const Transform &_trans,
										 float &validMovementFactor,
										 const Cylinder &cyl,
										 const RayInfo &movementRay,
										 glm::vec3 &normal) const
{
	RAMP_RECT_GLOB = *this;
	RAMP_RECT_GLOB_TRANS = _trans;
	
	printf("Ramp:  hw: %5.2f   hhs: %5.2f   hd: %5.2f   ht: %5.2f\n", 
			halfWidth, halfHeightSkewness, halfDepth, halfThickness);
			
// 	return RayTest(_trans, movementRay, validMovementFactor, normal);
	
	const float h2 = cyl.height * 0.5f;
	RampRectangle tmp{halfWidth + cyl.radius, halfHeightSkewness, halfDepth,
					  halfThickness + h2};
	Transform trans = _trans;
	trans.pos.y -= h2;
	return tmp.RayTest(trans, movementRay, validMovementFactor, normal);
}
} // namespace Collision3D
