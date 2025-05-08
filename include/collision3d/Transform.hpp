// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../SpatialPartitioning/glm/glm/ext/vector_float3.hpp"
#include "../../SpatialPartitioning/include/spatial_partitioning/RayInfo.hpp"

#include "Rotation.hpp"

namespace Collision3D
{
using namespace spp;

struct Transform {
	glm::vec3 pos;
	Rotation rot;

	// 	inline Transform inverse() const { return {-pos, rot.inverse()}; }

	inline Transform operator*(const Transform &r) const
	{
		return {(*this) * r.pos, rot + r.rot};
	}

	inline glm::vec3 operator*(const glm::vec3 &vec) const
	{
		return (rot * vec) + pos;
	}

	inline glm::vec2 operator*(const glm::vec2 &vec) const
	{
		return (rot * vec) + glm::vec2{pos.x, pos.z};
	}

	inline glm::vec3 ToLocal(const glm::vec3 &vec) const
	{
		return rot.ToLocal(vec - pos);
	}

	inline glm::vec2 ToLocal(const glm::vec2 &vec) const
	{
		return rot.ToLocal(vec - glm::vec2{pos.x, pos.z});
	}

	inline RayInfo ToLocal(const RayInfo &ray) const
	{
		RayInfo r2 = ray;
		r2.start = ray.start - pos;
		r2.dir = rot.ToLocal(ray.dir);
		r2.dirNormalized = rot.ToLocal(ray.dirNormalized);

		for (int i = 0; i < 3; i += 2) {
			r2.invDir[i] = r2.dir[i] == 0.0f ? 1e18f : 1.0f / r2.dir[i];
		}
		r2.invDir[1] = ray.invDir[1];

		r2.signs[0] = r2.invDir[0] < 0.0 ? 1 : 0;
		r2.signs[1] = ray.signs[1];
		r2.signs[2] = r2.invDir[2] < 0.0 ? 1 : 0;

		r2.end = r2.start + r2.dir;

		return r2;
	}
};
} // namespace Collision3D
