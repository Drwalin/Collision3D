// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

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

bool Sphere::RayTestLocal(const RayInfo &ray, float &near,
						  glm::vec3 &normal) const
{
	// TODO: warn because it is slower
	return sphereIntersect(ray, {}, radius, near, normal);
}

bool Sphere::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
								  glm::vec3 pos, float &offsetHeight,
								  glm::vec3 *onGroundNormal,
								  bool *isOnEdge) const
{
	return false;
	assert(!"Spheres only have ray tes, no movement test.");
	/*
	const glm::vec3 localPos = pos - trans.pos;
	const glm::vec3 localPos2d = {localPos.x, 0.0f, localPos.z};
	const float r2 = glm::length2(localPos2d);
	if (r2 > radius * radius) {
		return false;
	}

	const float y = sqrt(r2 + localPos.y * localPos.y);
	offsetHeight = localPos.y - y;

	if (onGroundNormal) {
		*onGroundNormal = glm::vec3{localPos2d.x, y, localPos2d.z} / radius;
	}

	return true;
	*/
}

/*
inline bool CapIntersect(glm::vec3 ro, glm::vec3 rd, glm::vec3 pa, float height,
				   float radius, float &near, glm::vec3 &normal)
{
	const glm::vec3 pb = pa + glm::vec3{0.0f, height, 0.0f};
	const glm::vec3 ba = pb - pa;
	const glm::vec3 oa = ro - pa;
	const float baba = glm::dot(ba, ba);
	const float bard = glm::dot(ba, rd);
	const float baoa = glm::dot(ba, oa);
	const float rdoa = glm::dot(rd, oa);
	const float oaoa = glm::dot(oa, oa);
	const float a = baba - bard * bard;
	float b = baba * rdoa - baoa * bard;
	float c = baba * oaoa - baoa * baoa - radius * radius * baba;
	float h = b * b - a * c;
	if (h >= 0.0f) {
		const float t = (-b - sqrt(h)) / a;
		const float y = baoa + t * bard;
		// body
		if (y > 0.0f && y < baba) {
			near = t;
		} else {
			// caps
			const glm::vec3 oc = (y <= 0.0f) ? oa : ro - pb;
			b = glm::dot(rd, oc);
			c = glm::dot(oc, oc) - radius * radius;
			h = b * b - c;
			if (h > 0.0f) {
				near = -b - sqrt(h);
			}
		}

		if (t > 1.0f) {
			// hit is too far
			return false;
		} else if (t < 0.0f) {
			// origin may be inside
			assert(!"Unimplemented");
			return true;
		} else {
			// hit is correct
			assert(!"Unimplemented");
			return true;
		}
	}
	return false;
}
*/

bool Sphere::CylinderTestMovement(const Transform &trans,
								  float &validMovementFactor,
								  const Cylinder &cyl,
								  const RayInfo &movementRay,
								  glm::vec3 &normal) const
{
	return false;
	assert(!"Spheres only have ray tes, no movement test.");
}
} // namespace Collision3D
