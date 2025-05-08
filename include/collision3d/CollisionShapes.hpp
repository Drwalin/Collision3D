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
template <typename T> struct HeightMap {
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

	// Treting cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()
};
extern template struct HeightMap<int8_t>;
extern template struct HeightMap<uint8_t>;
extern template struct HeightMap<int16_t>;
extern template struct HeightMap<uint16_t>;
extern template struct HeightMap<int32_t>;
extern template struct HeightMap<uint32_t>;
extern template struct HeightMap<float>;

// Origin at base
struct VerticalCappedCone {
	float baseRadius;
	float topRadius;
	float height;
};
} // namespace Collision3D
