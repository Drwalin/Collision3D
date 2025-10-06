// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes_AnyOrCompound.hpp"

namespace Collision3D
{
using namespace spp;

EACH_SHAPE(AnyShape, CONSTRUCTOR_SHAPE, EMPTY_CODE)

EACH_SHAPE(AnyShape, OPERATOR_SET_SHAPE, EMPTY_CODE)

AnyShape::AnyShape() { type = INVALID; }

AnyShape::AnyShape(AnyShape &other)
{
	this->type = other.type;
	this->trans = other.trans;
	switch (other.type) {
	case INVALID:
		break;
		EACH_SHAPE(AnyShape, SWITCH_CASES, SIMPLE_CODE_DO_COPY);
	default:
		type = INVALID;
	}
}

AnyShape::AnyShape(AnyShape &&other)
{
	this->type = other.type;
	this->trans = other.trans;
	switch (other.type) {
	case INVALID:
		break;
		EACH_SHAPE(AnyShape, SWITCH_CASES, SIMPLE_CODE_DO_MOVE);
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
		EACH_SHAPE(AnyShape, SWITCH_CASES, SIMPLE_CODE_DO_COPY);
	default:
		type = INVALID;
	}
}

AnyShape::~AnyShape()
{
	switch (type) {
		EACH_SHAPE(AnyShape, SWITCH_CASES, SIMPLE_CODE_CALL_DESTRUCTOR);
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

bool AnyShape::RayTestLocal(const RayInfo &ray, float &near,
							glm::vec3 &normal) const
{
	return RayTest({}, ray, near, normal);
}

bool AnyShape::CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,
									glm::vec3 pos, float &offsetHeight,
									glm::vec3 *onGroundNormal) const
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

#define CODE_COPY_FROM_ANY_PRIMITIVE(SHAPE, NAME, INDEX, DEREF)                \
	type = INDEX;                                                              \
	NAME = other.NAME;
AnyShape::AnyShape(AnyPrimitive &other)
{
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_COPY_FROM_ANY_PRIMITIVE);
	default:
	}
}

#define CODE_MOVE_FROM_ANY_PRIMITIVE(SHAPE, NAME, INDEX, DEREF)                \
	type = INDEX;                                                              \
	NAME = std::move(other.NAME);
AnyShape::AnyShape(const AnyPrimitive &other)
{
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_MOVE_FROM_ANY_PRIMITIVE);
	default:
	}
}

AnyShape::AnyShape(AnyPrimitive &&other)
{
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_MOVE_FROM_ANY_PRIMITIVE);
	default:
	}
}

#define CODE_COPY_FROM_ANY_PRIMITIVE(SHAPE, NAME, INDEX, DEREF)                \
	type = INDEX;                                                              \
	NAME = other.NAME;
AnyShape &AnyShape::operator=(AnyPrimitive &other)
{
	this->~AnyShape();
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_COPY_FROM_ANY_PRIMITIVE);
	default:
	}
	return *this;
}

#define CODE_MOVE_FROM_ANY_PRIMITIVE(SHAPE, NAME, INDEX, DEREF)                \
	type = INDEX;                                                              \
	NAME = std::move(other.NAME);
AnyShape &AnyShape::operator=(const AnyPrimitive &other)
{
	this->~AnyShape();
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_MOVE_FROM_ANY_PRIMITIVE);
	default:
	}
	return *this;
}

AnyShape &AnyShape::operator=(AnyPrimitive &&other)
{
	this->~AnyShape();
	trans = other.trans;
	switch (other.type) {
	case AnyPrimitive::INVALID:
		break;
		EACH_PRIMITIVE(AnyPrimitive, SWITCH_CASES,
					   CODE_MOVE_FROM_ANY_PRIMITIVE);
	default:
	}
	return *this;
}
} // namespace Collision3D
