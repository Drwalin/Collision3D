// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_Primitives.hpp"

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
	float bard = glm::dot(ba, ray.dirNormalized);
	float baoc = glm::dot(ba, oc);
	float k2 = baba - bard * bard;
	float k1 = baba * glm::dot(oc, ray.dirNormalized) - baoc * bard;
	float k0 = baba * glm::dot(oc, oc) - baoc * baoc - radius * radius * baba;
	float h = k1 * k1 - k2 * k0;
	if (h < 0.0)
		return false;
	h = sqrt(h);
	float t = (-k1 - h) / k2;
	float t2 = (-k1 + h) / k2;
	
	if (t < 0 && t2 > 0) { // is probably inside
		assert(glm::distance(glm::vec2{pos.x, pos.z}, {ray.start.x, ray.start.z}) <= radius * 1.01);
		if (ray.start.y >= 0 && ray.start.y <= height) {
			near = 0;
			const glm::vec3 outDir = (ray.start - pos) * glm::vec3(1,0,1);
			const float outDirLen = glm::length(outDir);
			if (outDirLen < 0.0000001) {
				normal = glm::vec3(1,0,0);
				return true;
			}
			
			normal = outDir / outDirLen;
			return true;
		}
	}

	// body
	float y = baoc + t * bard;
	if (y > 0.0 && y < baba) {
		normal = (oc + t * ray.dir - ba * y / baba) / radius;
		t /= ray.length;
		near = t;
		if (t <= 1.0f && t >= 0) {
			return true;
		}
		return false;
	}

	// caps
	t = (((y < 0.0) ? 0.0 : baba) - baoc) / bard;
	if (glm::abs(k1 + k2 * t) < h) {
		normal = ba * glm::sign(y) / (float)sqrt(baba);
		t /= ray.length;
		near = t;
		if (t <= 1.0f) {
			return true;
		}
	}
	return false;
}

bool Cylinder::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					   glm::vec3 &normal) const
{
	return cylinderIntersect(ray, trans.pos, height, radius, near, normal);
}

bool Cylinder::RayTestLocal(const RayInfo &ray, float &near,
							glm::vec3 &normal) const
{
	// TODO: warn because it is slower
	return cylinderIntersect(ray, {}, height, radius, near, normal);
}

bool Cylinder::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
									glm::vec3 pos, float &offsetHeight,
									glm::vec3 *onGroundNormal) const
{
	float r2 = radius + cyl.radius;
	r2 = r2 * r2;
	glm::vec2 diff = {pos.x - trans.pos.x, pos.z - trans.pos.z};
	if (r2 >= glm::length2(diff)) {
		CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
		if (onGroundNormal) {
			*onGroundNormal = {0,1,0};
		}
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
