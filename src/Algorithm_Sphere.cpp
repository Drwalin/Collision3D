// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb Sphere::GetAabb(const Transform &trans) const
{
	glm::vec3 min = trans.pos - radius;
	glm::vec3 max = trans.pos + radius;
	return {min, max};
}

static bool sphereIntersect(const RayInfo &ray, glm::vec3 pos, float radius,
							float &near, glm::vec3 &normal)
{
	glm::vec3 oc = ray.start - pos;
	float b = glm::dot(oc, ray.dir);
	glm::vec3 qc = oc - b * ray.dir;
	float h = radius * radius - glm::dot(qc, qc);
	if (h < 0.0) {
		return false;
	}
	h = sqrt(h);
	
	near = -b - h;
	glm::vec3 point = ray.start + ray.dir * near;
	normal = (point - pos) / radius;
	return true;
}

bool Sphere::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					 glm::vec3 &normal) const
{
	return sphereIntersect(ray, trans.pos, radius, near, normal);
}

bool Sphere::RayTestLocal(const Transform &trans, const RayInfo &ray,
						  const RayInfo &rayLocal, float &near,
						  glm::vec3 &normal) const
{
	// TODO: warn because it is slower
	if (sphereIntersect(rayLocal, trans.pos, radius, near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool Sphere::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
								  glm::vec3 pos, float &offsetHeight) const
{
	assert(!"Uniiimplemented!");
	return false;
}

bool Sphere::CylinderTestMovement(const Transform &trans,
								  float &validMovementFactor,
								  const Cylinder &cyl,
								  const RayInfo &movementRay,
								  glm::vec3 &normal) const
{
	assert(!"Uniiimplemented!");
	return false;
}
} // namespace Collision3D
