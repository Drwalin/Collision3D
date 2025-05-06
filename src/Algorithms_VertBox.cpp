// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb VertBox::GetAabb(const Transform &trans) const
{
	Rotation rot = trans.rot;
	if (rot.value >= 120) {
		rot.value -= 120;
	}
	if (rot.value >= 60) {
		rot.value += 180;
	}

	glm::vec3 min, max;
	min.y = trans.pos.y;
	max.y = trans.pos.y + halfExtents.y * 2.0f;

	const glm::vec2 x = rot * glm::vec2{halfExtents.x * 2.0f, 0};
	const glm::vec2 z = rot * glm::vec2{0, halfExtents.z * 2.0f};

	const glm::vec2 p = {0, 0};
	const glm::vec2 px = x;
	const glm::vec2 pz = z;
	const glm::vec2 pxz = x + z;

	min.x = glm::min(p.x, pz.x);
	min.z = glm::min(p.y, px.y);

	max.x = glm::max(pxz.x, px.x);
	max.z = glm::max(pxz.y, pz.y);

	const glm::vec3 half = {halfExtents.x, 0, halfExtents.z};

	return {min + half, max + half};
}

inline bool FastRayTest2(const glm::vec3 min, const glm::vec3 max,
						 const RayInfo &ray, float &near, glm::vec3 &normal)
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
					  glm::vec3 &normal)
{
	return RayTestFast(trans, trans.ToLocal(ray), near, normal);
}

bool VertBox::RayTestFast(const Transform &trans, const RayInfo &rayInverse,
						  float &near, glm::vec3 &normal)
{
	if (FastRayTest2(-halfExtents, halfExtents, rayInverse, near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool VertBox::CylinderTestOnGround(const Transform &trans, const Cyllinder &cyl,
								   glm::vec3 pos, float &offsetHeight)
{
	pos = trans.ToLocal(pos);
	if (pos.x > halfExtents.x || pos.z > halfExtents.y) {
		return false;
	}
	CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
	return true;
}

void VertBox::CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
													const Cyllinder &cyl,
													glm::vec3 pos,
													float &offsetHeight)
{
	offsetHeight = pos.y - trans.pos.y - halfExtents.y * 2.0f;
}

bool VertBox::CylinderTestMovement(const Transform &trans,
								   float &validMovementFactor,
								   const Cyllinder &cyl, glm::vec3 from,
								   glm::vec3 to, glm::vec3 &normal)
{
	RayInfo movementRay(from, to);
	return CylinderTestMovementFast(trans, validMovementFactor, cyl,
									movementRay, normal);
}

bool VertBox::CylinderTestMovementFast(const Transform &trans,
									   float &validMovementFactor,
									   const Cyllinder &cyl,
									   const RayInfo &movementRay,
									   glm::vec3 &normal)
{

	return CylinderTestMovementFastest(trans, validMovementFactor, cyl,
									   trans.ToLocal(movementRay), normal);
}

bool VertBox::CylinderTestMovementFastest(const Transform &trans,
										  float &validMovementFactor,
										  const Cyllinder &cyl,
										  const RayInfo &movementRayLocal,
										  glm::vec3 &normal)
{
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
