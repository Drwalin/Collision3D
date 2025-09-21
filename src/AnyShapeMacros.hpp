// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#define EACH_PRIMITIVE_TYPE(CLASS, MACRO, CODE)                                \
	MACRO(CLASS, CODE, ., VertBox, vertBox, VERTBOX)                           \
	MACRO(CLASS, CODE, ., Cylinder, cylinder, CYLINDER)                        \
	MACRO(CLASS, CODE, ., Sphere, sphere, SPHERE)                              \
	MACRO(CLASS, CODE, ., RampRectangle, rampRectangle, RAMP_RECTANGLE)        \
	MACRO(CLASS, CODE, ., VerticalTriangle, vertTriangle, VERTICAL_TRIANGLE)   \
	MACRO(CLASS, CODE, ., VerticalCappedCone, cappedCone, CAPPED_CONE)         \
	MACRO(CLASS, CODE, ., RampTriangle, rampTriangle, RAMP_TRIANGLE)

#define SWITCH_CASES(CLASS, CODE, DEREF, SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX)   \
	case TYPE_INDEX: {                                                         \
		CODE(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF)                        \
	} break;

#define EMPTY_CODE(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF)

#define CONSTRUCTOR_SHAPE(CLASS, CODE, DEREF, SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX) \
CLASS::CLASS(SHAPE_TYPE SHAPE_NAME, Transform trans) \
	: SHAPE_NAME(std::move(SHAPE_NAME)), trans(trans), type(TYPE_INDEX) {}

#define OPERATOR_SET_SHAPE(CLASS, CODE, DEREF, SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX) \
CLASS &CLASS::operator=(SHAPE_TYPE SHAPE_NAME) { \
	this->~CLASS(); \
	new (this) CLASS(std::move(SHAPE_NAME)); \
	trans = {}; \
	return *this; \
}

#define SIMPLE_CODE_OPERATOR_COPY(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) { \
	SHAPE_NAME = other.SHAPE_NAME; \
}

#define CODE_GET_AABB(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) \
		return SHAPE_NAME DEREF GetAabb(trans * this->trans);

#define CODE_RAY_TEST(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) \
		return SHAPE_NAME DEREF RayTest(trans * this->trans, ray, near, normal);

#define CODE_RAY_TEST_LOCAL(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) \
		return SHAPE_NAME DEREF RayTestLocal(trans * this->trans, ray, rayLocal, near, normal);

#define CODE_CYLINDER_TEST_ON_GROUND(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) \
		return SHAPE_NAME DEREF CylinderTestOnGround(trans * this->trans, cyl, pos, offsetHeight);

#define CODE_CYLINDER_TEST_MOVEMENT(SHAPE_TYPE, SHAPE_NAME, TYPE_INDEX, DEREF) \
		return SHAPE_NAME DEREF CylinderTestMovement(trans * this->trans, validMovementFactor, cyl, movementRay, normal);
