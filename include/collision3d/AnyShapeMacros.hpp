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
	MACRO(CLASS, CODE, ., RampRectangle, rampRectangle, RAMP_RECTANGLE)        \
	MACRO(CLASS, CODE, ., VerticalTriangle, vertTriangle, VERTICAL_TRIANGLE)   \
	MACRO(CLASS, CODE, ., RampTriangle, rampTriangle, RAMP_TRIANGLE)

#define SWITCH_CASES(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)                   \
	case INDEX: {                                                              \
		CODE(SHAPE, NAME, INDEX, DEREF)                                        \
	} break;

#define EMPTY_CODE(SHAPE, NAME, INDEX, DEREF)

#define CONSTRUCTOR_SHAPE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX) \
CLASS::CLASS(SHAPE &&NAME, Transform trans) \
	: NAME(std::move(NAME)), trans(trans), type(INDEX) {}

#define OPERATOR_SET_SHAPE(CLASS, CODE, DEREF, SHAPE, NAME, INDEX)             \
	CLASS &CLASS::operator=(SHAPE &&NAME)                                      \
	{                                                                          \
		this->~CLASS();                                                        \
		new (this) CLASS(std::move(NAME));                                     \
		trans = {};                                                            \
		return *this;                                                          \
	}

#define SIMPLE_CODE_DO_COPY(SHAPE, NAME, INDEX, DEREF)                         \
	NAME = other.NAME;

#define SIMPLE_CODE_DO_MOVE(SHAPE, NAME, INDEX, DEREF)                         \
	NAME = std::move(other.NAME);

#define SIMPLE_CODE_CALL_DESTRUCTOR(SHAPE, NAME, INDEX, DEREF)                 \
	NAME.~SHAPE();

#define CODE_GET_AABB(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF GetAabb(trans * this->trans);

#define CODE_RAY_TEST(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF RayTest(trans * this->trans, ray, near, normal);

#define CODE_RAY_TEST_LOCAL(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF RayTestLocal(trans * this->trans, ray, rayLocal, near, normal);

#define CODE_CYLINDER_TEST_ON_GROUND(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF CylinderTestOnGround(trans * this->trans, cyl, pos, offsetHeight);

#define CODE_CYLINDER_TEST_MOVEMENT(SHAPE, NAME, INDEX, DEREF) \
		return NAME DEREF CylinderTestMovement(trans * this->trans, validMovementFactor, cyl, movementRay, normal);

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
