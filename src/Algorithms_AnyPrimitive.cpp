// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
AnyPrimitive::AnyPrimitive(VertBox vertBox) : vertBox(vertBox), type(VERTBOX) {}
AnyPrimitive::AnyPrimitive(Cylinder cylinder)
	: cylinder(cylinder), type(CYLINDER)
{
}
AnyPrimitive::AnyPrimitive(Sphere sphere) : sphere(sphere), type(SPHERE) {}
AnyPrimitive::AnyPrimitive(Rectangle rectangle)
	: rectangle(rectangle), type(RECTANGLE)
{
}
AnyPrimitive::AnyPrimitive(VerticalTriangle vertTriangle)
	: vertTriangle(vertTriangle), type(VERTICAL_TRIANGLE)
{
}
AnyPrimitive::AnyPrimitive(VerticalCappedCone cappedCone)
	: cappedCone(cappedCone), type(CAPPED_CONE)
{
}

AnyPrimitive &AnyPrimitive::operator=(VertBox vertBox)
{
	new (this) AnyPrimitive(vertBox);
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(Cylinder cylinder)
{
	new (this) AnyPrimitive(cylinder);
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(Sphere sphere)
{
	new (this) AnyPrimitive(sphere);
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(Rectangle rectangle)
{
	new (this) AnyPrimitive(rectangle);
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(VerticalTriangle vertTriangle)
{
	new (this) AnyPrimitive(vertTriangle);
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(VerticalCappedCone cappedCone)
{
	new (this) AnyPrimitive(cappedCone);
	return *this;
}

spp::Aabb AnyPrimitive::GetAabb(const Transform &trans) const
{
	switch (type) {
	case INVALID:
		return spp::AABB_INVALID;
	case VERTBOX:
		return vertBox.GetAabb(trans);
	case CYLINDER:
		return cylinder.GetAabb(trans);
	case SPHERE:
		return sphere.GetAabb(trans);
	case RECTANGLE:
		return rectangle.GetAabb(trans);
	case VERTICAL_TRIANGLE:
		return vertTriangle.GetAabb(trans);
	case CAPPED_CONE:
		return cappedCone.GetAabb(trans);
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
	case VERTBOX:
		return vertBox.RayTest(trans, ray, near, normal);
	case CYLINDER:
		return cylinder.RayTest(trans, ray, near, normal);
	case SPHERE:
		return sphere.RayTest(trans, ray, near, normal);
	case RECTANGLE:
		return rectangle.RayTest(trans, ray, near, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTest(trans, ray, near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTest(trans, ray, near, normal);
	default:
		return false;
	}
}
bool AnyPrimitive::RayTestLocal(const Transform &trans, const RayInfo &ray,
								const RayInfo &rayLocal, float &near,
								glm::vec3 &normal) const
{
	switch (type) {
	case INVALID:
		return false;
	case VERTBOX:
		return vertBox.RayTestLocal(trans, ray, rayLocal, near, normal);
	case CYLINDER:
		return cylinder.RayTestLocal(trans, ray, rayLocal, near, normal);
	case SPHERE:
		return sphere.RayTestLocal(trans, ray, rayLocal, near, normal);
	case RECTANGLE:
		return rectangle.RayTestLocal(trans, ray, rayLocal, near, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTestLocal(trans, ray, rayLocal, near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTestLocal(trans, ray, rayLocal, near, normal);
	default:
		return false;
	}
}

bool AnyPrimitive::CylinderTestOnGround(const Transform &trans,
										const Cylinder &cyl, glm::vec3 pos,
										float &offsetHeight) const
{
	switch (type) {
	case INVALID:
		return false;
	case VERTBOX:
		return vertBox.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case CYLINDER:
		return cylinder.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case SPHERE:
		return sphere.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case RECTANGLE:
		return rectangle.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case CAPPED_CONE:
		return cappedCone.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
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
	case VERTBOX:
		return vertBox.CylinderTestMovement(trans, validMovementFactor, cyl,
											movementRay, normal);
	case CYLINDER:
		return cylinder.CylinderTestMovement(trans, validMovementFactor, cyl,
											 movementRay, normal);
	case SPHERE:
		return sphere.CylinderTestMovement(trans, validMovementFactor, cyl,
										   movementRay, normal);
	case RECTANGLE:
		return rectangle.CylinderTestMovement(trans, validMovementFactor, cyl,
											  movementRay, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestMovement(trans, validMovementFactor,
												 cyl, movementRay, normal);
	case CAPPED_CONE:
		return cappedCone.CylinderTestMovement(trans, validMovementFactor, cyl,
											   movementRay, normal);
	default:
		return false;
	}
}
} // namespace Collision3D
