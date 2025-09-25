// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

// RayTestGrid based on:
//    https://playtechs.blogspot.com/2007/03/raytracing-on-grid.html

#include <cstring>
#include <cstdlib>

#include <limits>

#include "../include/collision3d/CollisionShapes_HeightMapHeader.hpp"

namespace Collision3D
{
using namespace spp;

HeightMap_Header *HeightMap_Header::Allocate(glm::ivec2 resolution)
{
	HeightMap_Header header;
	memset(&header, 0, sizeof(HeightMap_Header));
	header.resolution = resolution;
	size_t bytes = sizeof(HeightMap_Header);
	size_t offsetHeight = bytes;
	bytes += (resolution.x * resolution.y) * sizeof(Type);
	size_t offsetMaterial = bytes;
	bytes += (resolution.x * resolution.y) * sizeof(MaterialType);

	void *ptr = malloc(bytes);
	HeightMap_Header *ret = new (ptr) HeightMap_Header(header);
	ret->heights = (Type *)((size_t)ptr + offsetHeight);
	ret->material = (MaterialType *)((size_t)ptr + offsetMaterial);
	return ret;
}

glm::ivec2 HeightMap_Header::ConvertGlobalPosToCoord(const Transform &trans,
													 glm::vec2 pos2d) const
{
	pos2d = trans.ToLocal(pos2d) * invScale.x;
	return pos2d + 0.5f;
}

glm::vec3 HeightMap_Header::ConvertToLocalPos(const Transform &trans,
											  glm::vec3 pos) const
{
	pos = trans.ToLocal(pos) * invScale;
	;
	return pos;
}

void HeightMap_Header::InitMeta(float horizontalScale, float verticalScale)
{
	scale = {horizontalScale, verticalScale, horizontalScale};
	invScale = 1.0f / scale;
	maxDh1 = horizontalScale / verticalScale;
	maxDh11 = (sqrt(2.0) * horizontalScale) / verticalScale;
	size = glm::vec2(resolution - 1) * horizontalScale;
}

bool HeightMap_Header::Update(glm::ivec2 coord, Type value)
{
	if (IsValidCoord(coord) == false) {
		return false;
	}
	heights[Id<false>(coord)] = value;
	return true;
}

template <bool SAFE>
HeightMap_Header::Type HeightMap_Header::Get(glm::ivec2 coord) const
{
	return heights[Id<true>(coord)];
}

bool HeightMap_Header::SetMaterial(glm::ivec2 coord, MaterialType value)
{
	if (IsValidCoord(coord) == false) {
		return false;
	}
	material[Id<true>(coord)] = value;
	return true;
}

template <bool SAFE>
HeightMap_Header::MaterialType
HeightMap_Header::GetMaterial(glm::ivec2 coord) const
{
	return material[Id<true>(coord)];
}

template <bool SAFE> size_t HeightMap_Header::Id(glm::ivec2 coord) const
{
	if constexpr (SAFE) {
		coord = ClampCoord(coord);
	}
	return ((size_t)coord.x) + ((size_t)resolution.x) * ((size_t)coord.y);
}

glm::ivec2 HeightMap_Header::ClampCoord(glm::ivec2 coord) const
{
	return glm::clamp(coord, glm::ivec2{0, 0}, resolution);
}

bool HeightMap_Header::IsValidCoord(glm::ivec2 coord) const
{
	return coord.x >= 0 && coord.y >= 0 && coord.x < resolution.x &&
		   coord.y < resolution.y;
}

bool HeightMap_Header::IsValidCell(glm::ivec2 coord) const
{
	return coord.x >= 0 && coord.y >= 0 && coord.x + 1 < resolution.x &&
		   coord.y + 1 < resolution.y;
}

spp::Aabb HeightMap_Header::GetAabb(const Transform &trans) const
{
	glm::vec2 a = trans * glm::vec2(0, 0);
	glm::vec2 b = trans * glm::vec2(resolution.x - 1, 0);
	glm::vec2 c = trans * glm::vec2(0, resolution.y - 1);
	glm::vec2 d = trans * glm::vec2(resolution - 1);
	glm::vec2 min = glm::min(a, glm::min(b, glm::min(c, d)));
	glm::vec2 max = glm::max(a, glm::max(b, glm::max(c, d)));
	return spp::Aabb{{min.x, trans.pos.y, min.y},
					 {max.x, trans.pos.y + MAX_HEIGHT, max.y}};
}

bool HeightMap_Header::RayTest(const Transform &trans, const RayInfo &ray,
							   float &near, glm::vec3 &normal) const
{
	return RayTestLocal(trans.ToLocal(ray), near, normal);
}

bool HeightMap_Header::RayTestLocal(const RayInfo &_ray, float &near,
									glm::vec3 &normal) const
{
	RayInfo ray = _ray;

	ray.dir *= scale;
	ray.dir *= scale;
	ray.length = glm::length(ray.dir);
	ray.start *= invScale;
	ray.end *= invScale;
	ray.dirNormalized = ray.dir / ray.length;

	assert(glm::distance(ray.end, ray.start + ray.dir) < 0.001f);

	near = 1.0f;

	if (ray.dir.x > 0) {
		if (ray.dir.z > 0) {
			if (!RayTestGrid<1, 1>(ray, near, normal))
				return false;
		} else if (ray.dir.z == 0) {
			if (!RayTestGrid<1, 0>(ray, near, normal))
				return false;
		} else {
			if (!RayTestGrid<1, -1>(ray, near, normal))
				return false;
		}
	} else if (ray.dir.x == 0) {
		if (ray.dir.z > 0) {
			if (!RayTestGrid<0, 1>(ray, near, normal))
				return false;
		} else if (ray.dir.z == 0) {
			if (!RayTestGrid<0, 0>(ray, near, normal))
				return false;
		} else {
			if (!RayTestGrid<0, -1>(ray, near, normal))
				return false;
		}
	} else {
		if (ray.dir.z > 0) {
			if (!RayTestGrid<-1, 1>(ray, near, normal))
				return false;
		} else if (ray.dir.z == 0) {
			if (!RayTestGrid<-1, 0>(ray, near, normal))
				return false;
		} else {
			if (!RayTestGrid<-1, -1>(ray, near, normal))
				return false;
		}
	}
	normal *= scale;
	normal = glm::normalize(normal);
	return true;
}

template <int DIR_SIGN_X, int DIR_SIGN_Z>
bool HeightMap_Header::RayTestGrid(const RayInfo &ray, float &near,
								   glm::vec3 &normal) const
{
	float dx = ray.dir.x;
	float dz = ray.dir.z;

	int x = int(floor(ray.start.x));
	int z = int(floor(ray.start.z));

	int n = 1;
	int x_inc, z_inc;
	float error;

	if constexpr (DIR_SIGN_X == 0) {
		x_inc = 0;
		error = std::numeric_limits<double>::infinity();
	} else if constexpr (DIR_SIGN_X > 0) {
		x_inc = 1;
		n += int(floor(ray.end.x)) - x;
		error = (floor(ray.start.x) + 1 - ray.start.x) * dz;
	} else {
		x_inc = -1;
		n += x - int(floor(ray.end.x));
		error = (ray.start.x - floor(ray.start.x)) * dz;
	}

	if constexpr (DIR_SIGN_Z == 0) {
		z_inc = 0;
		error -= std::numeric_limits<double>::infinity();
	} else if constexpr (DIR_SIGN_X > 0) {
		z_inc = 1;
		n += int(floor(ray.end.z)) - z;
		error -= (floor(ray.start.z) + 1 - ray.start.z) * dx;
	} else {
		z_inc = -1;
		n += z - int(floor(ray.end.z));
		error -= (ray.start.z - floor(ray.start.z)) * dx;
	}

	bool stopIterating = false;
	if (n == 0) {
		return RayTestCell<DIR_SIGN_X, DIR_SIGN_Z>(ray, near, normal, x, z,
												   stopIterating);
	}

	for (; n > 0; --n) {
		if (RayTestCell<DIR_SIGN_X, DIR_SIGN_Z>(ray, near, normal, x, z,
												stopIterating)) {
			return true;
		}
		if (stopIterating) {
			return false;
		}

		if (error > 0) {
			z += z_inc;
			error -= dx;
		} else {
			x += x_inc;
			error += dz;
		}
	}
	return false;
}

template <int DIR_SIGN_X, int DIR_SIGN_Z>
bool HeightMap_Header::RayTestCell(const RayInfo &ray, float &near,
								   glm::vec3 &normal, int x, int z,
								   bool &stopIterating) const
{
	if (!IsValidCell({x, z})) {
		return false;
	}

	const size_t id = Id<false>({x, z});
	const Type h00 = heights[id];
	const Type h10 = heights[id + 1];
	const Type h01 = heights[id + resolution.x];
	const Type h11 = heights[id + resolution.x + 1];

#ifndef COLLISION3D_HEIGHT_MAP_REMOVE_CELL_BOUNDARY_CHECK
	const float miny = glm::min(glm::min(h00, h01), glm::min(h10, h11));
	const float maxy = glm::max(glm::max(h00, h01), glm::max(h10, h11));

	float t1 = 0, t2 = 1;
	if constexpr (DIR_SIGN_X != 0) {
		float a = (x - ray.start.x) / ray.dir.x;
		float b = (x + 1 - ray.start.x) / ray.dir.x;
		t1 = glm::min(a, b);
		t2 = glm::max(a, b);
	}

	if constexpr (DIR_SIGN_Z != 0) {
		float a = (z - ray.start.z) / ray.dir.z;
		float b = (z + 1 - ray.start.z) / ray.dir.z;
		t1 = glm::max(t1, glm::min(a, b));
		t2 = glm::min(t1, glm::max(a, b));
	}

	if (t1 > 1.001f) {
		stopIterating = true;
		return false;
	}

	const float h1 = ray.start.y + ray.dir.y * t1;
	const float h2 = ray.start.y + ray.dir.y * t2;

	const float maxty = glm::max(h1, h2);
	const float minty = glm::min(h1, h2);

	if (maxty < miny || minty > maxy) {
		return false;
	}
#endif

	bool res = TriangleRayTest<true>(h00, h01, h11, x, z, ray, near, normal);
	float n;
	glm::vec3 no;
	if (TriangleRayTest<false>(h00, h10, h11, x, z, ray, n, no)) {
		if (n < near || res == false) {
			near = n;
			normal = no;
			return true;
		}
	}
	return res;
}

template <bool TOP_ELSE_DOWN>
bool HeightMap_Header::TriangleRayTest(Type h00, Type hxy, Type h11, int x,
									   int z, const RayInfo &localRay,
									   float &near, glm::vec3 &normal) const
{
	assert(x >= 0 || z >= 0 || x + 1 < resolution.x || z + 1 < resolution.y);
	const glm::vec3 v0{x, h00, z};
	const glm::vec3 v1 =
		TOP_ELSE_DOWN ? glm::vec3{x, hxy, z + 1} : glm::vec3{x + 1, hxy, z};
	const glm::vec3 v2{x + 1, h11, z + 1};
	const glm::vec3 rov0 = localRay.start - v0;

	// v2 - v0
	const glm::vec3 v1v0 = v1 - v0;
	const glm::vec3 v2v0{1, h11 - h00, 1};

	// glm::cross(v1v0, v2v0)
	const glm::vec3 n = {
		v1v0.y - v1v0.z * v2v0.y, //
		v1v0.z - v1v0.x,		  //
		v1v0.x * v2v0.y - v1v0.y  //
	};

	assert(glm::length(n - glm::cross(v1v0, v2v0)) < 0.000000001);

	const float d = 1.0f / glm::dot(localRay.dir, n);
	const float t = d * glm::dot(-n, rov0);

	const glm::vec3 hp = localRay.start + localRay.dir * t;

	if constexpr (TOP_ELSE_DOWN) {
		if (hp.x < 0.0f || 1.0f < hp.z || (hp.x - hp.z) > 0.0f) {
			return false;
		}
	} else {
		if (hp.z < 0.0f || 1.0f < hp.x || (hp.z - hp.x) > 0.0f) {
			return false;
		}
	}

	if (t < 0.0f || t > 1.0f) {
		return false;
	}

	near = t;
	normal = n;
	return true;
}

bool HeightMap_Header::CylinderTestOnGround(const Transform &trans,
											const Cylinder &cyl, glm::vec3 pos,
											float &offsetHeight) const
{
	pos = trans.ToLocal(pos) * invScale;
	int x = pos.x;
	int z = pos.z;
	float fracx = pos.x - x;
	float fracz = pos.z - z;
	if (!IsValidCell({x, z})) {
		return false;
	}

	const size_t id = Id<true>({x, z});
	Type a00 = heights[id];
	Type a11 = heights[id + resolution.x + 1];

	if (glm::abs(a00 - a11) > maxDh11) {
		return false;
	}

	float hdiag = a00 * (1.0f - fracz) + a11 * fracz;

	if (fracz > fracx) { // upper triangle
		float a01 = heights[id + resolution.x];

		if (glm::abs(a00 - a01) > maxDh1 || glm::abs(a11 - a01) > maxDh1) {
			return false;
		}

		float f = fracx / fracz;

		float hy = a00 * (1.0f - fracz) + a01 * fracz;
		float h = hy * (1.0f - f) + hdiag * f;

		offsetHeight = trans.pos.y - h;
		return true;
	} else { // lower triangle
		float a10 = heights[id + 1];

		if (glm::abs(a00 - a10) > maxDh1 || glm::abs(a11 - a10) > maxDh1) {
			return false;
		}

		if (fracx == 0.0f) {
			offsetHeight = trans.pos.y - a00;
			return true;
		}

		float f = (fracx - fracz) / (1.0f - fracz);

		float hy = a10 * (1.0f - fracz) + a11 * fracz;
		float h = hy * f + hdiag * (1.0f - f);

		offsetHeight = trans.pos.y - h;
		return true;
	}
}

bool HeightMap_Header::CylinderTestMovement(const Transform &trans,
											float &validMovementFactor,
											const Cylinder &cyl,
											const RayInfo &movementRay,
											glm::vec3 &normal) const
{
	if (RayTest(trans, movementRay, validMovementFactor, normal)) {
		return true;
	} else {
		validMovementFactor = 1.0f;
		return false;
	}
}

template HeightMap_Header::Type
HeightMap_Header::Get<true>(glm::ivec2 coord) const;
template HeightMap_Header::Type
HeightMap_Header::Get<false>(glm::ivec2 coord) const;
template HeightMap_Header::MaterialType
HeightMap_Header::GetMaterial<true>(glm::ivec2 coord) const;
template HeightMap_Header::MaterialType
HeightMap_Header::GetMaterial<false>(glm::ivec2 coord) const;
template size_t HeightMap_Header::Id<true>(glm::ivec2 coord) const;
template size_t HeightMap_Header::Id<false>(glm::ivec2 coord) const;
} // namespace Collision3D
