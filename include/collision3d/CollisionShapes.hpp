// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>

#include "HeightMapUtil.hpp"
#include "CollisionAlgorithms.hpp"

namespace Collision3D
{
using namespace spp;

// Origin at center of base
// Can be both wall and floor
struct VertBox {
	glm::vec3 halfExtents;

	// Collision treats cylinder as aligned square prism to tran
	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of base
// Can be both wall and floor
struct Cylinder {
	float height;
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
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
template <typename T> struct Heightmap {
	glm::vec3 scale;

	Matrix<T> points;
	std::vector<Matrix<PairMinMax<T>>> mipmap;

	void GenerateMipmap();
	void UpdateMipMap(int x, int y);

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at base
struct VerticalCappedCone {
	float baseRadius;
	float topRadius;
	float height;
};
} // namespace Collision3D
