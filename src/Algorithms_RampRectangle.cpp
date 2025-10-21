// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

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
			return false;
		}
	}

	if (far < 0.0f) {
		return false;
	}

	if (far < near) {
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
			return true; // frontface
		} else {
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

	return true;
}

bool RampRectangle::CylinderTestMovement(const Transform &_trans,
										 float &validMovementFactor,
										 const Cylinder &cyl,
										 const RayInfo &movementRay,
										 glm::vec3 &normal) const
{
	const float h2 = cyl.height * 0.5f;
	RampRectangle tmp{halfWidth + cyl.radius, halfHeightSkewness, halfDepth,
					  halfThickness + h2};
	Transform trans = _trans;
	trans.pos.y -= h2;
	return tmp.RayTest(trans, movementRay, validMovementFactor, normal);
}
} // namespace Collision3D
