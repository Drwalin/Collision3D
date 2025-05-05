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
struct VertBox {
	glm::vec3 halfExtents;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of base
struct Cyllinder {
	float height;
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center
struct Sphere {
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge
struct Rectangle {
	float width;
	float height;
	float depth;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct Triangle {
	glm::vec3 a, b, c;

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
} // namespace Collision3D
