// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

AnyShape::AnyShape(VertBox vertBox) : vertBox(vertBox), type(VERTBOX) {}
AnyShape::AnyShape(Cylinder cylinder) : cylinder(cylinder), type(CYLINDER) {}
AnyShape::AnyShape(Sphere sphere) : sphere(sphere), type(SPHERE) {}
AnyShape::AnyShape(Rectangle rectangle) : rectangle(rectangle), type(RECTANGLE)
{
}
AnyShape::AnyShape(VerticalTriangle vertTriangle)
	: vertTriangle(vertTriangle), type(VERTICAL_TRIANGLE)
{
}
AnyShape::AnyShape(VerticalCappedCone cappedCone)
	: cappedCone(cappedCone), type(CAPPED_CONE)
{
}
AnyShape::AnyShape(HeightMap<float, uint8_t> &&heightMap)
	: heightMap(new HeightMap<float, uint8_t>(std::move(heightMap))),
	  type(HEIGHT_MAP)
{
}
AnyShape::AnyShape(CompoundPrimitive &&compound)
	: compound(std::move(compound)), type(COMPOUND)
{
}

AnyShape &AnyShape::operator=(VertBox vertBox)
{
	new (this) AnyShape(vertBox);
	return *this;
}
AnyShape &AnyShape::operator=(Cylinder cylinder)
{
	new (this) AnyShape(cylinder);
	return *this;
}
AnyShape &AnyShape::operator=(Sphere sphere)
{
	new (this) AnyShape(sphere);
	return *this;
}
AnyShape &AnyShape::operator=(Rectangle rectangle)
{
	new (this) AnyShape(rectangle);
	return *this;
}
AnyShape &AnyShape::operator=(VerticalTriangle vertTriangle)
{
	new (this) AnyShape(vertTriangle);
	return *this;
}
AnyShape &AnyShape::operator=(VerticalCappedCone cappedCone)
{
	new (this) AnyShape(cappedCone);
	return *this;
}
AnyShape &AnyShape::operator=(HeightMap<float, uint8_t> &&heightMap)
{
	this->~AnyShape();
	new (this) AnyShape(std::move(heightMap));
	return *this;
}
AnyShape &AnyShape::operator=(CompoundPrimitive &&compound)
{
	this->~AnyShape();
	new (this) AnyShape(std::move(compound));
	return *this;
}

AnyShape::AnyShape()
{
	type = INVALID;
}

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
	case HEIGHT_MAP:
		return heightMap->GetAabb(trans);
	case COMPOUND:
		return compound.GetAabb(trans);
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
	case HEIGHT_MAP:
		return heightMap->RayTest(trans, ray, near, normal);
	case COMPOUND:
		return compound.RayTest(trans, ray, near, normal);
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
	case HEIGHT_MAP:
		return heightMap->RayTestLocal(trans, ray, rayLocal, near, normal);
	case COMPOUND:
		return compound.RayTestLocal(trans, ray, rayLocal, near, normal);
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
	case HEIGHT_MAP:
		return heightMap->CylinderTestOnGround(trans, cyl, pos, offsetHeight);
	case COMPOUND:
		return compound.CylinderTestOnGround(trans, cyl, pos, offsetHeight);
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
	case HEIGHT_MAP:
		return heightMap->CylinderTestMovement(trans, validMovementFactor, cyl,
											   movementRay, normal);
	case COMPOUND:
		return compound.CylinderTestMovement(trans, validMovementFactor, cyl,
											 movementRay, normal);
	default:
		return false;
	}
}
} // namespace Collision3D
