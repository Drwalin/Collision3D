// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb RampRectangle::GetAabb(const Transform &trans) const
{
	glm::vec2 a = trans.rot * glm::vec2{-halfWidth, 0};
	glm::vec2 b = trans.rot * glm::vec2{+halfWidth, depth};

	float c = 0.0f, d = height;
	if (d < c)
		std::swap(c, d);

	glm::vec3 min = trans.pos - glm::vec3{a.x, c - halfThickness, a.y};
	glm::vec3 max = trans.pos + glm::vec3{b.x, d + halfThickness, b.y};
	return {min, max};
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
	const glm::vec3 non = glm::normalize(glm::vec3{0, depth, -height});
	const float nl = glm::length(non);
	const glm::vec3 no = non / nl;
	const glm::vec3 n[6] = {{0, 0, -1}, {1, 0, 0}, {0, 0, 1},
							{-1, 0, 0}, no,		   -no};
	const float ofn = halfThickness * no.y;
	const float offs[6] = {0, halfWidth, depth, halfWidth, ofn, ofn};
	int frontNormalId = -1, backNormalId = -1;

	near = -1e9f;
	float far = 1e9f;

	for (int i = 0; i < 6; ++i) {
		if (TestPlaneIterational(n[i], offs[i], ray, near, far, frontNormalId,
								 backNormalId, i) == false) {
			return false;
		}
	}

	if (near >= 0.0f) {
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
										 float &offsetHeight) const
{
	if (fabs(height) > depth) {
		return false;
	}

	const glm::vec3 localPos = trans.ToLocal(pos);

	if (fabs(localPos.x) > halfWidth) {
		return false;
	}

	if (localPos.z < 0) {
		return false;
	}

	if (localPos.z > depth) {
		return false;
	}

	const float y = ((height + halfThickness) * localPos.z) / depth;
	offsetHeight = localPos.y - y;
	return true;
}

bool RampRectangle::CylinderTestMovement(const Transform &_trans,
										 float &validMovementFactor,
										 const Cylinder &cyl,
										 const RayInfo &movementRay,
										 glm::vec3 &normal) const
{
	const float h2 = cyl.height * 0.5f;
	RampRectangle tmp{halfWidth, height, depth, halfThickness + h2};
	Transform trans = _trans;
	trans.pos.y += h2;
	return tmp.RayTest(trans, movementRay, validMovementFactor, normal);
}
} // namespace Collision3D
