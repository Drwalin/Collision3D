// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>

#include "ForwardDeclarations.hpp"
#include "CollisionAlgorithms.hpp"
#include "AnyShapeMacros.hpp"

namespace Collision3D
{
namespace TypesShared
{
enum Enum : uint8_t {
	INVALID = 0,
	VERTBOX = 1,
	CYLINDER = 2,
	SPHERE = 3,
	RAMP_RECTANGLE = 4,
	VERTICAL_TRIANGLE = 5,
	RAMP_TRIANGLE = 6,
	HEIGHT_MAP = 62,
	COMPOUND = 63,
};
}

struct AnyPrimitive {
	union {
		EACH_PRIMITIVE(AnyPrimitive, DECLARATION_UNION_ANY_PRIMITIVE,
					   EMPTY_CODE)
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		EACH_PRIMITIVE(AnyPrimitive, DEFINITION_ENUM_VALUES, EMPTY_CODE)
	} type = INVALID;

	EACH_PRIMITIVE(AnyPrimitive, DECLARATION_CONSTRUCTORS_MOVE, EMPTY_CODE)

	AnyPrimitive() = default;

	AnyPrimitive(AnyPrimitive &other) = default;
	AnyPrimitive(AnyPrimitive &&other) = default;
	AnyPrimitive(const AnyPrimitive &other) = default;

	AnyPrimitive &operator=(AnyPrimitive &other) = default;
	AnyPrimitive &operator=(AnyPrimitive &&other) = default;
	AnyPrimitive &operator=(const AnyPrimitive &other) = default;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct CompoundPrimitive {
	std::vector<AnyPrimitive> primitives;

	CompoundPrimitive() = default;

	CompoundPrimitive(CompoundPrimitive &other) = default;
	CompoundPrimitive(CompoundPrimitive &&other) = default;
	CompoundPrimitive(const CompoundPrimitive &other) = default;

	CompoundPrimitive &operator=(CompoundPrimitive &other) = default;
	CompoundPrimitive &operator=(CompoundPrimitive &&other) = default;
	CompoundPrimitive &operator=(const CompoundPrimitive &other) = default;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct AnyShape {
	union {
		EACH_SHAPE(AnyShape, DECLARATION_UNION_ANY_PRIMITIVE, EMPTY_CODE)
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		EACH_SHAPE(AnyShape, DEFINITION_ENUM_VALUES, EMPTY_CODE)
	} type = INVALID;

	EACH_SHAPE(AnyShape, DECLARATION_CONSTRUCTORS_MOVE, EMPTY_CODE)

	AnyShape();
	~AnyShape();

	AnyShape(AnyShape &other);
	AnyShape(AnyShape &&other);
	AnyShape(const AnyShape &other);

	AnyShape &operator=(AnyShape &other);
	AnyShape &operator=(AnyShape &&other);
	AnyShape &operator=(const AnyShape &other);
	
	AnyShape(AnyPrimitive &other);
	AnyShape(const AnyPrimitive &other);
	AnyShape(AnyPrimitive &&other);
	
	AnyShape &operator=(AnyPrimitive &other);
	AnyShape &operator=(const AnyPrimitive &other);
	AnyShape &operator=(AnyPrimitive &&other);

	COLLISION_SHAPE_METHODS_DECLARATION()
};
} // namespace Collision3D
