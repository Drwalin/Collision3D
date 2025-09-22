// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"
	
namespace Collision3D
{
using namespace spp;

EACH_PRIMITIVE_OR_COMPOUND(AnyShape, CONSTRUCTOR_SHAPE, EMPTY_CODE)
AnyShape::AnyShape(HeightMap &&heightMap, Transform trans)
	: heightMap(new HeightMap(std::move(heightMap))),
	  trans(trans), type(HEIGHT_MAP) {}

EACH_PRIMITIVE(AnyShape, OPERATOR_SET_SHAPE, EMPTY_CODE)
OPERATOR_SET_SHAPE(AnyShape, {}, ->, HeightMap, heightMap, HEIGHT_MAP)
OPERATOR_SET_SHAPE(AnyShape, {}, ., CompoundPrimitive, compound, COMPOUND)
	

AnyShape::AnyShape() { type = INVALID; }

AnyShape::AnyShape(AnyShape &other)
{
	this->type = other.type;
	this->trans = other.trans;
	switch (other.type) {
	case INVALID: break;
	EACH_PRIMITIVE_OR_COMPOUND(AnyShape, SWITCH_CASES, SIMPLE_CODE_OPERATOR_COPY);
	case HEIGHT_MAP:
		heightMap = std::make_unique<HeightMap>(*other.heightMap);
		break;
	default:
		type = INVALID;
	}
}

AnyShape::AnyShape(AnyShape &&other)
{
	this->type = other.type;
	this->trans = other.trans;
	switch (other.type) {
	case INVALID: break;
	EACH_PRIMITIVE(AnyShape, SWITCH_CASES, SIMPLE_CODE_OPERATOR_COPY);
	case HEIGHT_MAP:
		heightMap = std::move(other.heightMap);
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
	this->trans = other.trans;
	switch (other.type) {
	case INVALID:
		break;
	EACH_PRIMITIVE(AnyShape, SWITCH_CASES, SIMPLE_CODE_OPERATOR_COPY);
	case COMPOUND:
		compound = other.compound;
		break;
	case HEIGHT_MAP:
		heightMap = std::make_unique<HeightMap>(*other.heightMap);
		break;
	default:
		type = INVALID;
	}
}

AnyShape::~AnyShape()
{
	switch (type) {
	case HEIGHT_MAP:
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
	EACH_SHAPE(AnyShape, SWITCH_CASES, CODE_GET_AABB);
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
	EACH_SHAPE(AnyShape, SWITCH_CASES, CODE_RAY_TEST);
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
	EACH_SHAPE(AnyShape, SWITCH_CASES, CODE_RAY_TEST_LOCAL);
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
	EACH_SHAPE(AnyShape, SWITCH_CASES, CODE_CYLINDER_TEST_ON_GROUND);
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
	EACH_SHAPE(AnyShape, SWITCH_CASES, CODE_CYLINDER_TEST_MOVEMENT);
	default:
		return false;
	}
}
} // namespace Collision3D
