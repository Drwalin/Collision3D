// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>
#include <memory>

#include "CollisionAlgorithms.hpp"
#include "AnyShapeMacros.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{

enum TypesShared : uint8_t {
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

struct AnyPrimitive {
	union {
		EACH_PRIMITIVE(AnyPrimitive, CODE_UNION_ANY_PRIMITIVE, EMPTY_CODE)
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		EACH_PRIMITIVE(AnyPrimitive, CODE_ENUM_VALUES, EMPTY_CODE)
	} type = INVALID;

	EACH_PRIMITIVE(AnyPrimitive, CODE_CONSTRUCTORS_MOVE, EMPTY_CODE)

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
		EACH_PRIMITIVE_OR_COMPOUND(AnyShape, CODE_UNION_ANY_PRIMITIVE,
								   EMPTY_CODE)
		std::unique_ptr<HeightMap> heightMap;
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		EACH_SHAPE(AnyShape, CODE_ENUM_VALUES, EMPTY_CODE)
	} type = INVALID;

	EACH_SHAPE(AnyShape, CODE_CONSTRUCTORS_MOVE, EMPTY_CODE)

	AnyShape();

	AnyShape(AnyShape &other);
	AnyShape(AnyShape &&other);
	AnyShape(const AnyShape &other);

	AnyShape &operator=(AnyShape &other);
	AnyShape &operator=(AnyShape &&other);
	AnyShape &operator=(const AnyShape &other);

	~AnyShape();

	COLLISION_SHAPE_METHODS_DECLARATION()
};
} // namespace Collision3D
