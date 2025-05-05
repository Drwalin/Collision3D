// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "CollisionAlgorithms.hpp"

namespace Collision3D
{
using namespace spp;

// One edge is horizontal
// 60 degree at each vertex at XZ plane
// Origin at center of horizontal edge
struct Triangle60 {
	float edge;
	float vertexHeight;
	
	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Edge between two 45 angle vertices is horizontal
// 90 degree at XZ plane at vertex opposite origin
// 45 degree at XZ plane on other tow vertices
// Origin at center of horizontal edge
struct Triangle90_45 {
	float edge;
	float vertexHeight;
	
	COLLISION_SHAPE_METHODS_DECLARATION()
};
} // namespace Collision3D
