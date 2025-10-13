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

	if (near <= 0.0f) {
		near = 0.0f;
		
		const glm::vec3 out[2] = {max - ray.start, min - ray.start};
		const float o[4] = {out[0].x, out[0].z, out[1].x, out[1].z};
		int id = 0;
		for (int i=1; i<4; ++i) {
			if (fabs(o[id]) > fabs(o[i])) {
				id = i;
			}
		}
		normal = {0,0,0};
		normal[(id%2)*2] = (id/2) ? -1 : 1;
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
	glm::vec3 he = halfExtents;
	glm::vec3 min = -he;
	glm::vec3 max = he;
	min.y += halfExtents.y;
	max.y += halfExtents.y;
	return FastRayTest2(min, max, ray, near, normal);
}

bool VertBox::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
								   glm::vec3 pos, float &offsetHeight,
								   glm::vec3 *onGroundNormal) const
{
// 	printf("                        Testing cylinder on vertbox: "
// 			"trans: %.2f %.2f %.2f,       "
// 			"pos: %.2f %.2f %.2f,         "
// 			"halfExt: %.2f %.2f %.2f,     "
// 			"cyl rad: %.2f   height: %.2f "
// 			"\n",
// 			trans.pos.x,
// 			trans.pos.y,
// 			trans.pos.z,
// 			pos.x,
// 			pos.y,
// 			pos.z,
// 			halfExtents.x,
// 			halfExtents.y,
// 			halfExtents.z,
// 			cyl.radius, cyl.height
// 			);
	pos = trans.ToLocal(pos);
	if (fabs(pos.x) > halfExtents.x+cyl.radius || fabs(pos.z) > halfExtents.z+cyl.radius) {
// 		printf("FAILED CYLINDER TEST ON GROUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		return false;
	}
	CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
	if (onGroundNormal) {
		*onGroundNormal = glm::vec3{0,1,0};
	}
	return true;
}

// This is in local space coordinates
void VertBox::CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
													const Cylinder &cyl,
													glm::vec3 pos,
													float &offsetHeight) const
{
	offsetHeight = pos.y - (halfExtents.y * 2.0f);
// 	printf("VertBox::CylinderTestOnGroundAssumeCollision2D:   pos.y: %f   trans.pos.y: %f    halfExtents.y: %f   (x2.0)       offsetHeight: %.3f\n",
// 	pos.y+trans.pos.y, trans.pos.y, halfExtents.y, offsetHeight);
}

bool VertBox::CylinderTestMovement(const Transform &trans,
								   float &validMovementFactor,
								   const Cylinder &cyl,
								   const RayInfo &movementRay,
								   glm::vec3 &normal) const
{
	RayInfo movementRayLocal = trans.ToLocal(movementRay);

	glm::vec3 he = halfExtents + glm::vec3(cyl.radius, 0, cyl.radius);
	glm::vec3 min = -he;
	glm::vec3 max = he;
	min.y += halfExtents.y;
	max.y += halfExtents.y;
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
