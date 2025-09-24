// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_AnyOrCompound.hpp"

namespace Collision3D
{
EACH_PRIMITIVE(AnyPrimitive, CONSTRUCTOR_SHAPE, EMPTY_CODE)
EACH_PRIMITIVE(AnyPrimitive, OPERATOR_SET_SHAPE, EMPTY_CODE)

spp::Aabb AnyPrimitive::GetAabb(const Transform &trans) const
{
	switch (type) {
	case INVALID:
		return spp::AABB_INVALID;
		EACH_PRIMITIVE(AnyShape, SWITCH_CASES, CODE_GET_AABB);
	default:
		return spp::AABB_INVALID;
	}
}

bool AnyPrimitive::RayTest(const Transform &trans, const RayInfo &ray,
						   float &near, glm::vec3 &normal) const
{
	switch (type) {
	case INVALID:
		return false;
		EACH_PRIMITIVE(AnyShape, SWITCH_CASES, CODE_RAY_TEST);
	default:
		return false;
	}
}

bool AnyPrimitive::RayTestLocal(const RayInfo &ray, float &near,
								glm::vec3 &normal) const
{
	return RayTest({}, ray, near, normal);
}

bool AnyPrimitive::CylinderTestOnGround(const Transform &trans,
										const Cylinder &cyl, glm::vec3 pos,
										float &offsetHeight) const
{
	switch (type) {
	case INVALID:
		return false;
		EACH_PRIMITIVE(AnyShape, SWITCH_CASES, CODE_CYLINDER_TEST_ON_GROUND);
	default:
		return false;
	}
}

bool AnyPrimitive::CylinderTestMovement(const Transform &trans,
										float &validMovementFactor,
										const Cylinder &cyl,
										const RayInfo &movementRay,
										glm::vec3 &normal) const
{
	switch (type) {
	case INVALID:
		return false;
		EACH_PRIMITIVE(AnyShape, SWITCH_CASES, CODE_CYLINDER_TEST_MOVEMENT);
	default:
		return false;
	}
}
} // namespace Collision3D
