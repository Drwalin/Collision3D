// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>
#include <memory>

#include "HeightMapUtil.hpp"
#include "CollisionAlgorithms.hpp"
#include "AnyShapeMacros.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{
// Origin at center of base
// Can be both wall and floor
struct VertBox {
	glm::vec3 halfExtents;

	// Collision treats cylinder as aligned square prism to trans
	COLLISION_SHAPE_METHODS_DECLARATION()
	CYLINDER_TEST_ON_GROUND_ASSUME_COLLISION2D()
};

// Origin at center of base
// Can be both wall and floor
struct Cylinder {
	float height;
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
	CYLINDER_TEST_ON_GROUND_ASSUME_COLLISION2D()
};

// Origin at center
// Ray only
struct Sphere {
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge (-x, +x), and extends
struct RampRectangle {
	float halfWidth;	 // expands (-x/2 ; +x/2)
	float height;		 // expands +y
	float depth;		 // expands +z
	float halfThickness; // thickens symmetrically y

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge (-x, +x)
struct RampTriangle {
	float sideLength;
	float heightOfVertexOnZ; // height of vertex at (0, Y, +z)

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct Triangle {
	glm::vec3 a, b, c;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at p0
// Normal without roatation: z=1
struct VerticalTriangle {
	glm::vec2 p1, p2;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center bottom of AABB
struct HeightMap {
	glm::vec3 size;
	glm::vec3 halfSize;
	glm::vec3 scale;	// .x === .z
	glm::vec3 invScale; // 1 / scale
	int width;
	int height;
	
	using T = float;
	using MT = uint8_t;

	// should mean > 46 degree
	T maxDh1;
	T maxDh11;

	// diagonal is between (x, y) and (x+1, y+1)

	std::vector<Matrix<T>> mipmap;
	Matrix<MT> material;

	glm::ivec2 GetClosestPointIfAny(glm::vec2 verticalPos);

	void InitSet(int width, int height, const glm::vec3 &scale,
				 const glm::vec3 &size, T *heights, MT *materials);

	void InitValues(int width, int height, const glm::vec3 &scale,
					const glm::vec3 &size);
	void GenerateMipmap();
	void Update(int x, int y, T value);
	T GetMax2x2(const Matrix<T> &mat, int x, int y) const;

	template <bool TOP_ELSE_DOWN>
	bool TriangleRayTest(T h00, T hxy, T h11, int x, int z,
						 const RayInfo &localRay, float &near,
						 glm::vec3 &localNormalUnnormalised) const;

	template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
	bool RayTestLocalNode(const RayInfo &rayLocal, float &near,
						  glm::vec3 &localNormalUnnormalised, int depth, int x,
						  int z) const;
	template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
	bool RayTestLocalNodeCallOrdered(const RayInfo &rayLocal, float &near,
									 glm::vec3 &localNormalUnnormalised,
									 int depth, int x, int z) const;

	HeightMap() : width(0), height(0) {}

	HeightMap(HeightMap &other) = default;
	HeightMap(HeightMap &&other) = default;
	HeightMap(const HeightMap &other) = default;

	HeightMap &operator=(HeightMap &other) = default;
	HeightMap &operator=(HeightMap &&other) = default;
	HeightMap &operator=(const HeightMap &other) = default;

	// Treting cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()
};

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
