// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>

#include "HeightMapUtil.hpp"
#include "CollisionAlgorithms.hpp"

namespace Collision3D
{
// Origin at center of base
// Can be both wall and floor
struct VertBox {
	glm::vec3 halfExtents;

	// Collision treats cylinder as aligned square prism to tran
	COLLISION_SHAPE_METHODS_DECLARATION()
	void CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
											   const Cylinder &cyl,
											   glm::vec3 pos,
											   float &offsetHeight) const;
};

// Origin at center of base
// Can be both wall and floor
struct Cylinder {
	float height;
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
	void CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
											   const Cylinder &cyl,
											   glm::vec3 pos,
											   float &offsetHeight) const;
};

// Origin at center
// Ray only
struct Sphere {
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge
struct Rectangle {
	float width;  // expands x
	float height; // expands y
	float depth;  // expands z

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
template <typename T, typename MT> struct HeightMap {
	glm::vec3 size;
	glm::vec3 halfSize;
	glm::vec3 scale;	// .x === .z
	glm::vec3 invScale; // 1 / scale
	int width;
	int height;

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
extern template struct HeightMap<int8_t, uint8_t>;
extern template struct HeightMap<uint8_t, uint8_t>;
extern template struct HeightMap<int16_t, uint8_t>;
extern template struct HeightMap<uint16_t, uint8_t>;
extern template struct HeightMap<int32_t, uint8_t>;
extern template struct HeightMap<uint32_t, uint8_t>;
extern template struct HeightMap<float, uint8_t>;

// Origin at base
struct VerticalCappedCone {
	float baseRadius;
	float topRadius;
	float height;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct AnyPrimitive {
	union {
		VertBox vertBox;
		Cylinder cylinder;
		Sphere sphere;
		Rectangle rectangle;
		VerticalTriangle vertTriangle;
		VerticalCappedCone cappedCone;
	};

	enum Type {
		INVALID = 0,
		VERTBOX = 1,
		CYLINDER = 2,
		SPHERE = 3,
		RECTANGLE = 4,
		VERTICAL_TRIANGLE = 5,
		CAPPED_CONE = 6,
	} type = INVALID;

	AnyPrimitive(VertBox vertBox);
	AnyPrimitive(Cylinder cylinder);
	AnyPrimitive(Sphere sphere);
	AnyPrimitive(Rectangle rectangle);
	AnyPrimitive(VerticalTriangle vertTriangle);
	AnyPrimitive(VerticalCappedCone cappedCone);

	AnyPrimitive &operator=(VertBox vertBox);
	AnyPrimitive &operator=(Cylinder cylinder);
	AnyPrimitive &operator=(Sphere sphere);
	AnyPrimitive &operator=(Rectangle rectangle);
	AnyPrimitive &operator=(VerticalTriangle vertTriangle);
	AnyPrimitive &operator=(VerticalCappedCone cappedCone);
	
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
	struct Shape {
		AnyPrimitive primitive;
		Transform transform;
	};

	std::vector<Shape> shapes;
	
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
		VertBox vertBox;
		Cylinder cylinder;
		Sphere sphere;
		Rectangle rectangle;
		VerticalTriangle vertTriangle;
		VerticalCappedCone cappedCone;
		HeightMap<float, uint8_t> *heightMap;
		CompoundPrimitive compound;
	};

	enum Type {
		INVALID = 0,
		VERTBOX = 1,
		CYLINDER = 2,
		SPHERE = 3,
		RECTANGLE = 4,
		VERTICAL_TRIANGLE = 5,
		CAPPED_CONE = 6,
		HEIGHT_MAP = 7,
		COMPOUND = 8,
	} type = INVALID;

	AnyShape(VertBox vertBox);
	AnyShape(Cylinder cylinder);
	AnyShape(Sphere sphere);
	AnyShape(Rectangle rectangle);
	AnyShape(VerticalTriangle vertTriangle);
	AnyShape(VerticalCappedCone cappedCone);
	AnyShape(HeightMap<float, uint8_t> &&heightMap);
	AnyShape(CompoundPrimitive &&compound);

	AnyShape &operator=(VertBox vertBox);
	AnyShape &operator=(Cylinder cylinder);
	AnyShape &operator=(Sphere sphere);
	AnyShape &operator=(Rectangle rectangle);
	AnyShape &operator=(VerticalTriangle vertTriangle);
	AnyShape &operator=(VerticalCappedCone cappedCone);
	AnyShape &operator=(HeightMap<float, uint8_t> &&heightMap);
	AnyShape &operator=(CompoundPrimitive &&compound);
	
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
