// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
spp::Aabb CompoundPrimitive::GetAabb(const Transform &trans) const
{
	spp::Aabb aabb = spp::AABB_INVALID;
	for (const auto &s : primitives) {
		aabb = aabb + s.GetAabb(trans);
	}
	return aabb;
}

bool CompoundPrimitive::RayTest(const Transform &trans, const RayInfo &ray,
								float &near, glm::vec3 &normal) const
{
	bool res = false;
	float ne;
	glm::vec3 no;
	for (const auto &s : primitives) {
		if (s.RayTest(trans, ray, ne, no)) {
			if (res) {
				if (near > ne) {
					near = ne;
					normal = no;
				}
			} else {
				near = ne;
				normal = no;
				res = true;
			}
		}
	}
	return res;
}

bool CompoundPrimitive::RayTestLocal(const Transform &trans, const RayInfo &ray,
									 const RayInfo &rayLocal, float &near,
									 glm::vec3 &normal) const
{
	bool res = false;
	float ne;
	glm::vec3 no;
	for (const auto &s : primitives) {
		if (s.RayTest(trans, ray, ne, no)) {
			if (res) {
				if (near > ne) {
					near = ne;
					normal = no;
				}
			} else {
				near = ne;
				normal = no;
				res = true;
			}
		}
	}
	if (res) {
		normal = trans.rot * normal;
	}
	return res;
}

bool CompoundPrimitive::CylinderTestOnGround(const Transform &trans,
											 const Cylinder &cyl, glm::vec3 pos,
											 float &offsetHeight) const
{
	bool res = false;
	float ofh;
	for (const auto &s : primitives) {
		if (s.CylinderTestOnGround(trans, cyl, pos, ofh)) {
			if (res) {
				if (offsetHeight < ofh) {
					offsetHeight = ofh;
				}
			} else {
				offsetHeight = ofh;
				res = true;
			}
		}
	}
	return res;
}

bool CompoundPrimitive::CylinderTestMovement(const Transform &trans,
											 float &validMovementFactor,
											 const Cylinder &cyl,
											 const RayInfo &movementRay,
											 glm::vec3 &normal) const
{
	bool res = false;
	float vmf;
	glm::vec3 no;
	for (const auto &s : primitives) {
		if (s.CylinderTestMovement(trans, vmf, cyl, movementRay, no)) {
			if (res) {
				if (validMovementFactor > vmf) {
					validMovementFactor = vmf;
					normal = no;
				}
			} else {
				validMovementFactor = vmf;
				normal = no;
				res = true;
			}
		}
	}
	return res;
}
} // namespace Collision3D
