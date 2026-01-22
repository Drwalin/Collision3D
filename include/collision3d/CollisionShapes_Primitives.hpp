// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "./CollisionAlgorithms.hpp"

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

// Origin at center and extends
struct RampRectangle {
	float halfWidth;		  // expands (-x/2 ; +x/2)
	float halfHeightSkewness; // expands (-y/2 ; +y/2)
	float halfDepth;		  // expands (-z/2 ; +z/2)
	float halfThickness;	  // thickens symmetrically y

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Width is always 2, unless it's used with scale
// Origin at center of first horizontal edge (-1, +1)
// Two vertices are always at (-1, 0, 0) and (1, 0, 0)
// third vertex is at (vert3x, heightSkewness, depth)
struct RampTriangle {
	glm::vec3 p3;		 // z > 0
	float halfThickness; // thickens symmetrically y

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Infinitelly thin vertical triangle
// Origin at p0
// Normal without roatation: z=1
struct VerticalTriangle {
	glm::vec2 p1, p2; // {x, y} ; z=0

	COLLISION_SHAPE_METHODS_DECLARATION()
};
} // namespace Collision3D
