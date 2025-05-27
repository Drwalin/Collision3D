// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../SpatialPartitioning/glm/glm/ext/vector_float2.hpp"
#include "../../SpatialPartitioning/glm/glm/ext/vector_float3.hpp"

namespace Collision3D
{
struct Rotation {
	uint8_t value;

	inline Rotation inverse() const { return diff({0}, *this); }
	inline Rotation operator-(int) const { return inverse(); }

	inline Rotation add(Rotation r) const { return sum(*this, r); }
	inline Rotation sub(Rotation r) const { return diff(*this, r); }

	inline Rotation operator+(Rotation r) const { return add(r); }
	inline Rotation operator-(Rotation r) const { return sub(r); }

	inline Rotation operator*(short mult) const
	{
		int v = value;
		v *= mult;
		v %= 240;
		v += v < 0 ? 240 : 0;
		assert(v >= 0 && v < 240);
		return {(uint8_t)v};
	}

	inline float ToDegrees() const { return (360.0f / 240.0f) * value; }

	inline float ToRadians() const
	{
		return (3.1415926535897932384626f * 2.0f / 240.0f) * value;
	}

	inline static Rotation FromRadians(float radians)
	{
		int v =
			(radians * (240.0f / (3.1415926535897932384626f * 2.0f))) + 0.5f;
		v %= 240;
		v += v < 0 ? 240 : 0;
		assert(v >= 0 && v < 240);
		return {(uint8_t)v};
	}

	inline static Rotation FromDegrees(float degrees)
	{
		int v = (degrees * (240.0f / (360.0f))) + 0.5f;
		v %= 240;
		v += v < 0 ? 240 : 0;
		assert(v >= 0 && v < 240);
		return {(uint8_t)v};
	}

	inline glm::vec2 ToLocal(const glm::vec2 &v) const
	{
		assert(value < 240);
		const Rotation inv{(uint8_t)(240u - value)};
		glm::vec2 rot = inv.GetVec2();
		return glm::vec2{rot.x * v.x - rot.y * v.y, rot.y * v.x + rot.x * v.y};
	}

	inline glm::vec3 ToLocal(glm::vec3 v) const
	{
		assert(value < 240);
		const Rotation inv{(uint8_t)(240u - value)};
		glm::vec2 rot = inv.GetVec2();
		return glm::vec3{rot.x * v.x + rot.y * v.z, v.y,
						 -rot.y * v.x + rot.x * v.z};
	}

	inline glm::vec2 operator*(const glm::vec2 &v) const
	{
		assert(value < 240);
		glm::vec2 rot = GetVec2();
		return glm::vec2{rot.x * v.x - rot.y * v.y, rot.y * v.x + rot.x * v.y};
	}

	inline glm::vec3 operator*(glm::vec3 v) const
	{
		assert(value < 240);
		glm::vec2 rot = GetVec2();
		return glm::vec3{rot.x * v.x + rot.y * v.z, v.y,
						 -rot.y * v.x + rot.x * v.z};
	}

	inline static Rotation diff(Rotation l, Rotation r)
	{
		assert(r.value < 240);
		assert(r.value < 240);
		const int a = l.value;
		const int b = r.value;
		const int sum = a - b + (a > b ? 0 : 240);
		assert(sum >= 0 && sum <= 240);
		return {(uint8_t)sum};
	}

	inline static Rotation sum(Rotation l, Rotation r)
	{
		assert(r.value < 240);
		assert(r.value < 240);
		const int a = l.value;
		const int b = r.value;
		int sum = a + b;
		sum -= sum > 240 ? 240 : 0;
		assert(sum >= 0 && sum < 240);
		return {(uint8_t)sum};
	}

	const static glm::vec2 vecs[241]; // cos, sin
	inline glm::vec2 GetVec2() const
	{
		assert(value <= 240);
		return vecs[value];
	}
};
} // namespace Collision3D
