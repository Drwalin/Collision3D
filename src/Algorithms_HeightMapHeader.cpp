// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstring>
#include <cstdlib>

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

	if (!ray.signs[0]) {
		if (!ray.signs[2]) {
			if (!RayTestGrid<true, true>(ray, near, normal))
				return false;
		} else {
			if (!RayTestGrid<true, false>(ray, near, normal))
				return false;
		}
	} else {
		if (!ray.signs[2]) {
			if (!RayTestGrid<false, true>(ray, near, normal))
				return false;
		} else {
			if (!RayTestGrid<false, false>(ray, near, normal))
				return false;
		}
	}
	normal *= scale;
	normal = glm::normalize(normal);
	return true;
}

template <bool IS_POSITIVE>
static void Helper(float pos, float dir, float &tile, float &dtile, float &dt,
				   float &ddt)
{
	tile = floor(pos) + 1;
	if constexpr (IS_POSITIVE) {
		dtile = 1;
		dt = ((tile + 0) - pos) / dir;
	} else {
		dtile = -1;
		dt = ((tile - 1) - pos) / dir;
	}
	ddt = dtile / dir;
}

template <bool POSITIVE_DIR_X, bool POSITIVE_DIR_Z>
bool HeightMap_Header::RayTestGrid(const RayInfo &ray, float &near,
								   glm::vec3 &normal) const
{
	glm::vec2 dir2 = {ray.dir.x, ray.dir.z};
	float tilex, dtilex, dtx, ddtx, tilez, dtilez, dtz, ddtz;
	Helper<POSITIVE_DIR_X>(ray.start.x, ray.dir.x, tilex, dtilex, dtx, ddtx);
	Helper<POSITIVE_DIR_Z>(ray.start.z, ray.dir.z, tilez, dtilez, dtz, ddtz);
	float t = 0.0f;

	if (ray.dir.x * ray.dir.x + ray.dir.z * ray.dir.z > 0) {
		while (t <= 1.0f && tilex > 0 && tilex <= resolution.x && tilez > 0 &&
			   tilez <= resolution.y) {
			float oldx = tilex, oldz = tilez, oldt = t;
			if (dtx < dtz) {
				tilex += dtilex;
				const float dt = dtx;
				t += dt;
				dtx += ddtx - dt;
				dtz -= dt;
			} else {
				tilez += dtilez;
				const float dt = dtz;
				t += dt;
				dtx -= dt;
				dtz += ddtz - dt;
			}

			if (RayTestCell(ray, near, normal, oldx, oldz, oldt, t)) {
				return true;
			}
		}
		return false;
	} else {
		return RayTestCell(ray, near, normal, tilex, tilez, 0.0f, 1.0f);
	}
}

bool HeightMap_Header::RayTestCell(const RayInfo &ray, float &near,
								   glm::vec3 &normal, int x, int z, float t,
								   float nextt) const
{
	if (!(IsValidCoord({x, z}) && IsValidCoord({x + 1, z + 1}))) {
		return false;
	}

	Type h[2][2];
	size_t id = Id<true>({x, z});
	h[0][0] = heights[id];
	h[1][0] = heights[id + 1];
	h[0][1] = heights[id + resolution.x];
	h[1][1] = heights[id + resolution.x + 1];

	float miny = h[0][0];
	float maxy = h[0][0];
	miny = glm::min(miny, h[0][1]);
	miny = glm::min(miny, h[1][0]);
	miny = glm::min(miny, h[1][1]);
	maxy = glm::max(maxy, h[0][1]);
	maxy = glm::max(maxy, h[1][0]);
	maxy = glm::max(maxy, h[1][1]);

	float a = ray.dir.y * t;
	float b = ray.dir.y * nextt;

	float rminy = glm::min(a, b) + ray.start.y;
	float rmaxy = glm::max(a, b) + ray.start.y;

	if (rminy > maxy || rmaxy < miny) {
		return false;
	}

	bool res = TriangleRayTest<true>(h[0][0], h[0][1], h[1][1], x, z, ray, near,
									 normal);
	float n;
	glm::vec3 no;
	if (TriangleRayTest<false>(h[0][0], h[1][0], h[1][1], x, z, ray, n, no)) {
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
	if (!(IsValidCoord({x, z}) && IsValidCoord({x + 1, z + 1}))) {
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
