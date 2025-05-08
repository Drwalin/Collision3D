// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../SpatialPartitioning/glm/glm/ext/vector_float3.hpp"
#include "../../SpatialPartitioning/glm/glm/ext/vector_float2.hpp"
#include "../../SpatialPartitioning/glm/glm/geometric.hpp"

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

inline float dot2(const glm::vec3 &v) { return glm::dot(v, v); }
inline float dot2(const glm::vec2 &v) { return glm::dot(v, v); }
} // namespace glm
