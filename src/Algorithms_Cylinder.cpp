// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb Cylinder::GetAabb(const Transform &trans) const
{
	glm::vec3 min = trans.pos - glm::vec3{radius, 0, radius};
	glm::vec3 max = trans.pos + glm::vec3{radius, height, radius};
	return {min, max};
}

static bool cylinderIntersect(const RayInfo &ray, glm::vec3 pos, float height,
							  float radius, float &near, glm::vec3 &normal)
{
	glm::vec3 ba = {0, height, 0};
	glm::vec3 oc = ray.start - pos;
	float baba = glm::dot(ba, ba);
	float bard = glm::dot(ba, ray.dir);
	float baoc = glm::dot(ba, oc);
	float k2 = baba - bard * bard;
	float k1 = baba * glm::dot(oc, ray.dir) - baoc * bard;
	float k0 = baba * glm::dot(oc, oc) - baoc * baoc - radius * radius * baba;
	float h = k1 * k1 - k2 * k0;
	if (h < 0.0)
		return false;
	h = sqrt(h);
	float t = (-k1 - h) / k2;

	// body
	float y = baoc + t * bard;
	if (y > 0.0 && y < baba) {
		normal = (oc + t * ray.dir - ba * y / baba) / radius;
		near = t;
		return true;
	}

	// caps
	t = (((y < 0.0) ? 0.0 : baba) - baoc) / bard;
	if (glm::abs(k1 + k2 * t) < h) {
		normal = ba * glm::sign(y) / (float)sqrt(baba);
		near = t;
		return true;
	}
	return false;
}

bool Cylinder::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					   glm::vec3 &normal) const
{
	return cylinderIntersect(ray, trans.pos, height, radius, near, normal);
}

bool Cylinder::RayTestLocal(const Transform &trans, const RayInfo &ray,
							const RayInfo &rayLocal, float &near,
							glm::vec3 &normal) const
{
	// TODO: warn because it is slower
	if (cylinderIntersect(rayLocal, trans.pos, height, radius, near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool Cylinder::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
									glm::vec3 pos, float &offsetHeight) const
{
	float r2 = radius + cyl.radius;
	r2 = r2 * r2;
	glm::vec2 diff = {pos.x - trans.pos.x, pos.z - trans.pos.z};
	if (r2 >= glm::length2(diff)) {
		CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
		return true;
	}
	return false;
}

void Cylinder::CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
													 const Cylinder &cyl,
													 glm::vec3 pos,
													 float &offsetHeight) const
{
	offsetHeight = pos.y - trans.pos.y - height;
}

bool Cylinder::CylinderTestMovement(const Transform &trans,
									float &validMovementFactor,
									const Cylinder &cyl,
									const RayInfo &movementRay,
									glm::vec3 &normal) const
{
	Cylinder cyl2 = {height + cyl.height, radius + cyl.radius};
	return cylinderIntersect(
		movementRay, trans.pos - glm::vec3(0, cyl.height, 0), cyl2.height,
		cyl2.radius, validMovementFactor, normal);
}
} // namespace Collision3D
