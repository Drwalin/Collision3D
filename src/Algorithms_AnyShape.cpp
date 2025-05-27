// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

AnyShape::AnyShape(VertBox vertBox, Transform trans)
	: vertBox(vertBox), trans(trans), type(VERTBOX)
{
}
AnyShape::AnyShape(Cylinder cylinder, Transform trans)
	: cylinder(cylinder), trans(trans), type(CYLINDER)
{
}
AnyShape::AnyShape(Sphere sphere, Transform trans)
	: sphere(sphere), trans(trans), type(SPHERE)
{
}
AnyShape::AnyShape(Rectangle rectangle, Transform trans)
	: rectangle(rectangle), trans(trans), type(RECTANGLE)
{
}
AnyShape::AnyShape(VerticalTriangle vertTriangle, Transform trans)
	: vertTriangle(vertTriangle), trans(trans), type(VERTICAL_TRIANGLE)
{
}
AnyShape::AnyShape(VerticalCappedCone cappedCone, Transform trans)
	: cappedCone(cappedCone), trans(trans), type(CAPPED_CONE)
{
}
AnyShape::AnyShape(HeightMap<float, uint8_t> &&heightMap, Transform trans)
	: heightMap(new HeightMap<float, uint8_t>(std::move(heightMap))),
	  trans(trans), type(HEIGHT_MAP)
{
}
AnyShape::AnyShape(CompoundPrimitive &&compound, Transform trans)
	: compound(std::move(compound)), trans(trans), type(COMPOUND)
{
}

AnyShape &AnyShape::operator=(VertBox vertBox)
{
	this->~AnyShape();
	new (this) AnyShape(vertBox);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(Cylinder cylinder)
{
	this->~AnyShape();
	new (this) AnyShape(cylinder);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(Sphere sphere)
{
	this->~AnyShape();
	new (this) AnyShape(sphere);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(Rectangle rectangle)
{
	this->~AnyShape();
	new (this) AnyShape(rectangle);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(VerticalTriangle vertTriangle)
{
	this->~AnyShape();
	new (this) AnyShape(vertTriangle);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(VerticalCappedCone cappedCone)
{
	this->~AnyShape();
	new (this) AnyShape(cappedCone);
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(HeightMap<float, uint8_t> &&heightMap)
{
	this->~AnyShape();
	new (this) AnyShape(std::move(heightMap));
	trans = {};
	return *this;
}
AnyShape &AnyShape::operator=(CompoundPrimitive &&compound)
{
	this->~AnyShape();
	new (this) AnyShape(std::move(compound));
	trans = {};
	return *this;
}

AnyShape::AnyShape() { type = INVALID; }

AnyShape::AnyShape(AnyShape &other)
{
	this->type = other.type;
	switch (other.type) {
	case INVALID:
		break;
	case VERTBOX:
		vertBox = other.vertBox;
		break;
	case CYLINDER:
		cylinder = other.cylinder;
		break;
	case SPHERE:
		sphere = other.sphere;
		break;
	case RECTANGLE:
		rectangle = other.rectangle;
		break;
	case VERTICAL_TRIANGLE:
		vertTriangle = other.vertTriangle;
		break;
	case CAPPED_CONE:
		cappedCone = other.cappedCone;
		break;
	case HEIGHT_MAP:
		heightMap = new HeightMap<float, uint8_t>(*other.heightMap);
		break;
	case COMPOUND:
		compound = other.compound;
		break;
	default:
		type = INVALID;
	}
	this->trans = other.trans;
}

AnyShape::AnyShape(AnyShape &&other)
{
	this->type = other.type;
	switch (other.type) {
	case INVALID:
		break;
	case VERTBOX:
		vertBox = other.vertBox;
		break;
	case CYLINDER:
		cylinder = other.cylinder;
		break;
	case SPHERE:
		sphere = other.sphere;
		break;
	case RECTANGLE:
		rectangle = other.rectangle;
		break;
	case VERTICAL_TRIANGLE:
		vertTriangle = other.vertTriangle;
		break;
	case CAPPED_CONE:
		cappedCone = other.cappedCone;
		break;
	case HEIGHT_MAP:
		heightMap = other.heightMap;
		other.heightMap = nullptr;
		break;
	case COMPOUND:
		compound = std::move(other.compound);
		other.compound.~CompoundPrimitive();
		break;
	default:
		type = INVALID;
	}
	other.type = INVALID;
	this->trans = other.trans;
}

AnyShape::AnyShape(const AnyShape &other)
{
	this->type = other.type;
	switch (other.type) {
	case INVALID:
		break;
	case VERTBOX:
		vertBox = other.vertBox;
		break;
	case CYLINDER:
		cylinder = other.cylinder;
		break;
	case SPHERE:
		sphere = other.sphere;
		break;
	case RECTANGLE:
		rectangle = other.rectangle;
		break;
	case VERTICAL_TRIANGLE:
		vertTriangle = other.vertTriangle;
		break;
	case CAPPED_CONE:
		cappedCone = other.cappedCone;
		break;
	case HEIGHT_MAP:
		heightMap = new HeightMap<float, uint8_t>(*other.heightMap);
		break;
	case COMPOUND:
		compound = other.compound;
		break;
	default:
		type = INVALID;
	}
	this->trans = other.trans;
}

AnyShape::~AnyShape()
{
	switch (type) {
	case HEIGHT_MAP:
		delete heightMap;
		heightMap = nullptr;
		break;
	case COMPOUND:
		compound.~CompoundPrimitive();
		break;
	default:
	}
	type = INVALID;
}

AnyShape &AnyShape::operator=(AnyShape &other)
{
	this->~AnyShape();
	new (this) AnyShape(other);
	return *this;
}

AnyShape &AnyShape::operator=(AnyShape &&other)
{
	this->~AnyShape();
	new (this) AnyShape(std::move(other));
	return *this;
}

AnyShape &AnyShape::operator=(const AnyShape &other)
{
	this->~AnyShape();
	new (this) AnyShape(other);
	return *this;
}

spp::Aabb AnyShape::GetAabb(const Transform &trans) const
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
	case RECTANGLE:
		return rectangle.GetAabb(trans * this->trans);
	case VERTICAL_TRIANGLE:
		return vertTriangle.GetAabb(trans * this->trans);
	case CAPPED_CONE:
		return cappedCone.GetAabb(trans * this->trans);
	case HEIGHT_MAP:
		return heightMap->GetAabb(trans * this->trans);
	case COMPOUND:
		return compound.GetAabb(trans * this->trans);
	default:
		return spp::AABB_INVALID;
	}
}

bool AnyShape::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					   glm::vec3 &normal) const
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
	case RECTANGLE:
		return rectangle.RayTest(trans * this->trans, ray, near, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTest(trans * this->trans, ray, near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTest(trans * this->trans, ray, near, normal);
	case HEIGHT_MAP:
		return heightMap->RayTest(trans * this->trans, ray, near, normal);
	case COMPOUND:
		return compound.RayTest(trans * this->trans, ray, near, normal);
	default:
		return false;
	}
}

bool AnyShape::RayTestLocal(const Transform &trans, const RayInfo &ray,
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
	case RECTANGLE:
		return rectangle.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									  normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.RayTestLocal(trans * this->trans, ray, rayLocal,
										 near, normal);
	case CAPPED_CONE:
		return cappedCone.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									   normal);
	case HEIGHT_MAP:
		return heightMap->RayTestLocal(trans * this->trans, ray, rayLocal, near,
									   normal);
	case COMPOUND:
		return compound.RayTestLocal(trans * this->trans, ray, rayLocal, near,
									 normal);
	default:
		return false;
	}
}

bool AnyShape::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
									glm::vec3 pos, float &offsetHeight) const
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
	case RECTANGLE:
		return rectangle.CylinderTestOnGround(trans * this->trans, cyl, pos,
											  offsetHeight);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestOnGround(trans * this->trans, cyl, pos,
												 offsetHeight);
	case CAPPED_CONE:
		return cappedCone.CylinderTestOnGround(trans * this->trans, cyl, pos,
											   offsetHeight);
	case HEIGHT_MAP:
		return heightMap->CylinderTestOnGround(trans * this->trans, cyl, pos,
											   offsetHeight);
	case COMPOUND:
		return compound.CylinderTestOnGround(trans * this->trans, cyl, pos,
											 offsetHeight);
	default:
		return false;
	}
}

bool AnyShape::CylinderTestMovement(const Transform &trans,
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
	case RECTANGLE:
		return rectangle.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case VERTICAL_TRIANGLE:
		return vertTriangle.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case CAPPED_CONE:
		return cappedCone.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case HEIGHT_MAP:
		return heightMap->CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	case COMPOUND:
		return compound.CylinderTestMovement(
			trans * this->trans, validMovementFactor, cyl, movementRay, normal);
	default:
		return false;
	}
}
} // namespace Collision3D
