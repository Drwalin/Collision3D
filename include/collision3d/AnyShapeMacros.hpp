// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "CollisionShapes_Primitives.hpp"
#include "CollisionShapes_HeightMap.hpp"

#define EACH_PRIMITIVE(CLASS, MACRO, CODE)                                     \
	MACRO(CLASS, CODE, ., VertBox, vertBox, VERTBOX)                           \
	MACRO(CLASS, CODE, ., Cylinder, cylinder, CYLINDER)                        \
	MACRO(CLASS, CODE, ., Sphere, sphere, SPHERE)                              \
	MACRO(CLASS, CODE, ., RampRectangle, rampRectangle, RAMP_RECTANGLE)
// 	MACRO(CLASS, CODE, ., VerticalTriangle, vertTriangle, VERTICAL_TRIANGLE)   \
// 	MACRO(CLASS, CODE, ., RampTriangle, rampTriangle, RAMP_TRIANGLE)

#define SWITCH_CASES(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)                   \
	case CLASS::INDEX: {                                                       \
		CODE(SHAPE, NAME, INDEX, DEREF)                                        \
	} break;

#define EMPTY_CODE(SHAPE, NAME, INDEX, DEREF)

#define CONSTRUCTOR_SHAPE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX) \
CLASS::CLASS(SHAPE &&NAME, Transform trans) \
	: NAME(std::move(NAME)), pos(trans.pos), rot(trans.rot), type(INDEX) {}

#define OPERATOR_SET_SHAPE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)             \
	CLASS &CLASS::operator=(SHAPE &&NAME)                                      \
	{                                                                          \
		this->~CLASS();                                                        \
		new (this) CLASS(std::move(NAME));                                     \
		pos = {};                                                              \
		rot = {};                                                              \
		return *this;                                                          \
	}

#define SIMPLE_CODE_DO_COPY(SHAPE, NAME, INDEX, DEREF)                         \
	NAME = other.NAME;

#define SIMPLE_CODE_DO_MOVE(SHAPE, NAME, INDEX, DEREF)                         \
	NAME = std::move(other.NAME);

#define SIMPLE_CODE_CALL_DESTRUCTOR(SHAPE, NAME, INDEX, DEREF)                 \
	NAME.~SHAPE();

#define CODE_GET_AABB(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF GetAabb(trans * Transform{pos, rot});

#define CODE_RAY_TEST(SHAPE, NAME, INDEX, DEREF)                               \
	if (NAME DEREF RayTest(trans * Transform{pos, rot}, ray, near, normal)) {  \
		normal = (trans.rot + rot) * normal;                                   \
		return true;                                                           \
	} else {                                                                   \
		return false;                                                          \
	}

#define CODE_CYLINDER_TEST_ON_GROUND(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF CylinderTestOnGround(trans * Transform{pos, rot}, cyl, pos, offsetHeight, onGroundNormal);

#define CODE_CYLINDER_TEST_MOVEMENT(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF CylinderTestMovement(trans * Transform{pos, rot}, validMovementFactor, cyl, movementRay, normal);

#define EACH_SHAPE(CLASS, MACRO, CODE)                                         \
	EACH_PRIMITIVE(CLASS, MACRO, CODE)                                         \
	MACRO(CLASS, CODE, ., CompoundPrimitive, compound, COMPOUND)               \
	MACRO(CLASS, CODE, ., HeightMap, heightMap, HEIGHT_MAP)

#define DEFINITION_ENUM_VALUES(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)               \
	INDEX = TypesShared::Enum::INDEX,

#define DECLARATION_UNION_ANY_PRIMITIVE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)       \
	SHAPE NAME;

#define DECLARATION_CONSTRUCTORS_MOVE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)  \
	CLASS(SHAPE &&NAME, Transform trans = {});                                 \
	CLASS &operator=(SHAPE &&NAME);


