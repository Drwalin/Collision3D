// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
AnyPrimitive::AnyPrimitive(VertBox vertBox, Transform trans)
	: vertBox(vertBox), trans(trans), type(VERTBOX)
{
}
AnyPrimitive::AnyPrimitive(Cylinder cylinder, Transform trans)
	: cylinder(cylinder), trans(trans), type(CYLINDER)
{
}
AnyPrimitive::AnyPrimitive(Sphere sphere, Transform trans)
	: sphere(sphere), trans(trans), type(SPHERE)
{
}
AnyPrimitive::AnyPrimitive(RampRectangle rectangle, Transform trans)
	: rectangle(rectangle), trans(trans), type(RAMP_RECTANGLE)
{
}
AnyPrimitive::AnyPrimitive(VerticalTriangle vertTriangle, Transform trans)
	: vertTriangle(vertTriangle), trans(trans), type(VERTICAL_TRIANGLE)
{
}
AnyPrimitive::AnyPrimitive(VerticalCappedCone cappedCone, Transform trans)
	: cappedCone(cappedCone), trans(trans), type(CAPPED_CONE)
{
}
AnyPrimitive::AnyPrimitive(RampTriangle rampTriangle, Transform trans)
	: rampTriangle(rampTriangle), trans(trans), type(RAMP_TRIANGLE)
{
}

AnyPrimitive &AnyPrimitive::operator=(VertBox vertBox)
{
	new (this) AnyPrimitive(vertBox);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(Cylinder cylinder)
{
	new (this) AnyPrimitive(cylinder);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(Sphere sphere)
{
	new (this) AnyPrimitive(sphere);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(RampRectangle rectangle)
{
	new (this) AnyPrimitive(rectangle);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(VerticalTriangle vertTriangle)
{
	new (this) AnyPrimitive(vertTriangle);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(VerticalCappedCone cappedCone)
{
	new (this) AnyPrimitive(cappedCone);
	trans = {};
	return *this;
}
AnyPrimitive &AnyPrimitive::operator=(RampTriangle rampTriangle)
{
	new (this) AnyPrimitive(rampTriangle);
	trans = {};
	return *this;
}

spp::Aabb AnyPrimitive::GetAabb(const Transform &trans) const
{
	switch (type) {
	case INVALID:
		return spp::AABB_INVALID;
	case VERTBOX:
		return vertBox.GetAabb(trans * this->trans);
	case CYLINDER:
		return cylinder.GetAabb(trans * this->trans);
	case SPHERE:
		return sphere.GetAabb(trans * this->trans);
	case RAMP_RECTANGLE:
		return rectangle.GetAabb(trans * this->trans);
	case VERTICAL_TRIANGLE:
		return vertTriangle.GetAabb(trans * this->trans);
	case CAPPED_CONE:
		return cappedCone.GetAabb(trans * this->trans);
	case RAMP_TRIANGLE:
		return rampTriangle.GetAabb(trans * this->trans);
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
		return vertBox.RayTest(trans * this->trans, ray, near, normal);
	case CYLINDER:
		return cylinder.RayTest(trans * this->trans, ray, near, normal);
	case SPHERE:
		return sphere.RayTest(trans * this->trans, ray, near, normal);
	case RAMP_RECTANGLE:
		return rectangle.RayTest(trans * this->trans, ray, near, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTest(trans * this->trans, ray, near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTest(trans * this->trans, ray, near, normal);
	case RAMP_TRIANGLE:
		return rampTriangle.RayTest(trans * this->trans, ray, near, normal);
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
		return vertBox.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									normal);
	case CYLINDER:
		return cylinder.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									 normal);
	case SPHERE:
		return sphere.RayTestLocal(trans * this->trans, ray, rayLocal, near,
								   normal);
	case RAMP_RECTANGLE:
		return rectangle.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									  normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTestLocal(trans * this->trans, ray, rayLocal,
										 near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									   normal);
	case RAMP_TRIANGLE:
		return rampTriangle.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									   normal);
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
		return vertBox.CylinderTestOnGround(trans * this->trans, cyl, pos,
											offsetHeight);
	case CYLINDER:
		return cylinder.CylinderTestOnGround(trans * this->trans, cyl, pos,
											 offsetHeight);
	case SPHERE:
		return sphere.CylinderTestOnGround(trans * this->trans, cyl, pos,
										   offsetHeight);
	case RAMP_RECTANGLE:
		return rectangle.CylinderTestOnGround(trans * this->trans, cyl, pos,
											  offsetHeight);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestOnGround(trans * this->trans, cyl, pos,
												 offsetHeight);
	case CAPPED_CONE:
		return cappedCone.CylinderTestOnGround(trans * this->trans, cyl, pos,
											   offsetHeight);
	case RAMP_TRIANGLE:
		return rampTriangle.CylinderTestOnGround(trans * this->trans, cyl, pos,
											   offsetHeight);
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
		return vertBox.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case CYLINDER:
		return cylinder.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case SPHERE:
		return sphere.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case RAMP_RECTANGLE:
		return rectangle.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case CAPPED_CONE:
		return cappedCone.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case RAMP_TRIANGLE:
		return rampTriangle.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	default:
		return false;
	}
}
} // namespace Collision3D
