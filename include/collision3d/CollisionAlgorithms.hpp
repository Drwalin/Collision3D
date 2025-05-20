// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../SpatialPartitioning/glm/glm/ext/vector_int2.hpp"

#include "../../SpatialPartitioning/include/spatial_partitioning/RayInfo.hpp"
#include "../../SpatialPartitioning/include/spatial_partitioning/Aabb.hpp"

#include "Transform.hpp"
#include "MathUtil.hpp"

namespace Collision3D
{
using namespace spp;

struct Cylinder;
} // namespace Collision3D

#define COLLISION_SHAPE_METHODS_DECLARATION()                                  \
	spp::Aabb GetAabb(const Transform &trans) const;                           \
	bool RayTest(const Transform &trans, const RayInfo &ray, float &near,      \
				 glm::vec3 &normal) const;                                     \
	bool RayTestLocal(const Transform &trans, const RayInfo &ray,              \
					  const RayInfo &rayLocal, float &near, glm::vec3 &normal) \
		const;                                                                 \
	bool CylinderTestMovement(const Transform &trans,                          \
							  float &validMovementFactor, const Cylinder &cyl, \
							  const RayInfo &movementRay, glm::vec3 &normal)   \
		const;                                                                 \
	bool CylinderTestOnGround(const Transform &trans, const Cylinder &cyl,     \
							  glm::vec3 pos, float &offsetHeight) const;
