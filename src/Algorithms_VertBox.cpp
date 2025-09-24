// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb VertBox::GetAabb(const Transform &trans) const
{
	Rotation rot = trans.rot;
	if (rot.value >= 120) {
		rot.value -= 120;
	}

	glm::vec3 min = trans.pos, max = trans.pos;
	min.y = trans.pos.y;
	max.y += halfExtents.y * 2.0f;

	const glm::vec2 x = rot * glm::vec2{halfExtents.x, 0};
	const glm::vec2 z = rot * glm::vec2{0, halfExtents.z};
	
	const glm::vec2 a = glm::abs(x + z);
	const glm::vec2 b = glm::abs(x - z);
	const glm::vec2 c = glm::abs(- x + z);
	const glm::vec2 d = glm::abs(- x - z);
	
	const glm::vec2 min2 = glm::max(a, glm::max(b, glm::max(c, d)));
	
	min.x -= min2.x;
	min.z -= min2.y;
	max.x += min2.x;
	max.z += min2.y;
	
	return {min, max};
}

static inline bool FastRayTest2(const glm::vec3 min, const glm::vec3 max,
								const RayInfo &ray, float &near,
								glm::vec3 &normal)
{
	assert(glm::all(glm::lessThanEqual(min, max)));
	alignas(16) glm::vec3 bounds[2] = {min, max};
	alignas(16) glm::vec3 tmin, tmax;

	int normalAxis = 0;

	for (int i = 0; i < 2; ++i) {
		tmin[i] = (bounds[ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
		tmax[i] = (bounds[1 - ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
	}

	if ((tmin.x > tmax.y) || (tmin.y > tmax.x))
		return false;

	if (tmin.y > tmin.x) {
		tmin.x = tmin.y;
		normalAxis = 1;
	}

	if (tmax.y < tmax.x) {
		tmax.x = tmax.y;
	}

	for (int i = 2; i < 3; ++i) {
		tmin[i] = (bounds[ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
		tmax[i] = (bounds[1 - ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
	}

	if ((tmin.x > tmax.z) || (tmin.z > tmax.x)) {
		return false;
	}

	if (tmin.z > tmin.x) {
		normalAxis = 2;
		tmin.x = tmin.z;
	}
	if (tmax.z < tmax.x) {
		tmax.x = tmax.z;
	}
	near = tmin.x;
	float far = tmax.x;

	if (far < 0.0f)
		return false;

	if (near > far)
		return false;

	if (near < 0.0f) {
		normal = -ray.dirNormalized;
		near = 0.0f;
	} else {
		normal = {0, 0, 0};
		normal[normalAxis] = ray.signs[normalAxis] ? 1 : -1;
		assert(glm::dot(normal, ray.dir) < 0);
	}

	return true;
}

bool VertBox::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					  glm::vec3 &normal) const
{
	if (RayTestLocal(trans.ToLocal(ray), near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool VertBox::RayTestLocal(const RayInfo &ray, float &near,
						   glm::vec3 &normal) const
{
	return FastRayTest2(-halfExtents, halfExtents, ray, near, normal);
}

bool VertBox::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
								   glm::vec3 pos, float &offsetHeight) const
{
	pos = trans.ToLocal(pos);
	if (fabs(pos.x) > halfExtents.x || fabs(pos.z) > halfExtents.y) {
		return false;
	}
	CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
	return true;
}

void VertBox::CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
													const Cylinder &cyl,
													glm::vec3 pos,
													float &offsetHeight) const
{
	offsetHeight = pos.y - trans.pos.y - halfExtents.y * 2.0f;
}

bool VertBox::CylinderTestMovement(const Transform &trans,
								   float &validMovementFactor,
								   const Cylinder &cyl,
								   const RayInfo &movementRay,
								   glm::vec3 &normal) const
{
	RayInfo movementRayLocal = trans.ToLocal(movementRay);

	glm::vec3 he = halfExtents;
	glm::vec3 min = -he;
	glm::vec3 max = he;
	min.y -= cyl.height;
	if (FastRayTest2(min, max, movementRayLocal, validMovementFactor, normal)) {
		if (validMovementFactor > 1.0f) {
			validMovementFactor = 1.0f;
			return false;
		}
		assert(validMovementFactor >= 0.0f);
		normal = trans.rot * normal;
		return true;
	} else {
		validMovementFactor = 1.0f;
		return false;
	}
}
} // namespace Collision3D
