// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../SpatialPartitioning/include/spatial_partitioning/RayInfo.hpp"
#include "../../SpatialPartitioning/include/spatial_partitioning/Aabb.hpp"

#include "Transform.hpp"

namespace Collision3D
{
using namespace spp;

struct Cyllinder;
} // namespace Collision3D

#define COLLISION_SHAPE_METHODS_DECLARATION()                                  \
	spp::Aabb GetAabb(const Transform &trans) const;                           \
	bool RayTest(const Transform &trans, const RayInfo &ray, float &near,      \
				 glm::vec3 &normal);                                           \
	bool RayTestFast(const Transform &trans, const RayInfo &rayInverse,        \
					 float &near, glm::vec3 &normal);                          \
	bool CylinderTestMovement(const Transform &trans,                          \
							  float &validMovementFactor,                      \
							  const Cyllinder &cyl, glm::vec3 from,            \
							  glm::vec3 to, glm::vec3 &normal);                \
	bool CylinderTestOnGround(const Transform &trans, const Cyllinder &cyl,    \
							  glm::vec3 pos, float &offsetHeight);             \
	bool CylinderTestOnGroundAssumeCollision2D(                                \
		const Transform &trans, const Cyllinder &cyl, glm::vec3 pos,           \
		float &offsetHeight);

namespace glm
{
inline float maxcomp(const glm::vec2 &v) { return glm::max(v.x, v.y); }
inline float mincomp(const glm::vec2 &v) { return glm::min(v.x, v.y); }
inline float maxcomp(const glm::vec3 &v)
{
	return glm::max(v.x, glm::max(v.y, v.z));
}
inline float mincomp(const glm::vec3 &v)
{
	return glm::min(v.x, glm::min(v.y, v.z));
}
inline float length2(const glm::vec3 &v) { return glm::dot(v, v); }
inline float length2(const glm::vec2 &v) { return glm::dot(v, v); }
} // namespace glm
