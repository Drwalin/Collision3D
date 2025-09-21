// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

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
	return RayTestLocal(trans, ray, trans.ToLocal(ray), near, normal);
}

bool RampRectangle::RayTestLocal(const Transform &trans, const RayInfo &ray,
								 const RayInfo &rayLocal, float &near,
								 glm::vec3 &normal) const
{
	const glm::vec3 non = glm::normalize(glm::vec3{0, depth, -height});
	const float nl = glm::length(non);
	const glm::vec3 no = non / nl;
	const glm::vec3 n[6] = {{0, 0, -1}, {1, 0, 0}, {0, 0, 1},
							{-1, 0, 0}, no,		   -no};
	const float ofn = halfThickness * no.y;
	const float offs[6] = {0, halfWidth, depth, halfWidth, ofn, ofn};

	near = -1e9;
	float far = 1e9;

	for (int i = 0; i < 6; ++i) {
		bool useNormal = false;
		if (TestPlaneIterational(n[i], offs[i], rayLocal, near, far,
								 useNormal)) {
			if (useNormal) {
				normal = n[i];
			}
		} else {
			return false;
		}
	}

	if (near >= far) {
		return false;
	}

	if (far <= 0.0f) {
		return false;
	}

	if (near < 0.0f) {
		near = 0.0f;
		return true;
	} else if (near > 1) {
		return false;
	} else {
		return true;
	}
}

bool RampRectangle::CylinderTestOnGround(const Transform &trans,
										 const Cylinder &cyl, glm::vec3 pos,
										 float &offsetHeight) const
{
	float r2 = radius + cyl.radius;
	r2 = r2 * r2;
	glm::vec2 diff = {pos.x - trans.pos.x, pos.y - trans.pos.y};
	if (r2 >= glm::length2(diff)) {
		RampRectangleTestOnGroundAssumeCollision2D(trans, cyl, pos,
												   offsetHeight);
		return true;
	}
	return false;
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
