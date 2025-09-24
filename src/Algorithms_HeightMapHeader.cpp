// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstring>
#include <cstdlib>

#include "../include/collision3d/CollisionShapes_Primitives.hpp"
#include "../include/collision3d/CollisionShapes_HeightMapHeader.hpp"

namespace Collision3D
{
using namespace spp;

HeightMap_Header *HeightMap_Header::Allocate(int resolution)
{
	assert(((resolution-1) & (resolution - 2)) == 0 && "resolution needs to be power of two");
	
	HeightMap_Header header;
	memset(&header, 0, sizeof(HeightMap_Header));
	int res = resolution;
	header.resolution = resolution;
	size_t bytes = sizeof(HeightMap_Header);
	header.heights = (Type*)bytes;
	bytes += res * res * sizeof(Type);
	res -= 1;
	header.resMipmap = res;
	for (int i = 0; i < 16 && res; ++i, res/=2) {
		assert((res > 1 && res > 1));
		header.heightsMipmap[i] = (Type *)bytes;
		const size_t s = res * res * sizeof(Type);
		bytes += s;
		++header.levels;
	}
	header.material = (MaterialType *)bytes;
	bytes += res * res * sizeof(MaterialType);
	header.bytes = bytes;

	HeightMap_Header *ret = new (malloc(bytes)) HeightMap_Header(header);

	for (int i = 0; i < header.levels; ++i) {
		*((size_t*)&(header.heightsMipmap[i])) +=(size_t)ret;
	}
	*((size_t*)&(header.heights)) +=(size_t)ret;
	header.material = (MaterialType *)(((size_t)header.material) + (size_t)ret);

	return ret;
}

glm::ivec2 HeightMap_Header::ConvertGlobalPosToCoord(const Transform &trans,
		glm::vec3 pos) const
{
	pos = trans.ToLocal(pos) * invScale;
	return pos + 0.5f;
}

void HeightMap_Header::InitMeta(float scale)
{
	this->scale = scale;
	invScale = 1.0f / scale;
	maxDh1 = 1.0f;
	maxDh11 = sqrt(2.0f);
	horizontalSize = scale * (resolution-1);
}

void HeightMap_Header::GenerateMipmap()
{
	for (size_t y = 0, ida = 0, idb = 0; y < resMipmap; ++y, ++ida) {
		for (int x = 0; x < resMipmap; ++x, ++idb, ++ida) {
			Type v = heights[ida];
			v = glm::max<Type>(v, heights[ida+1]);
			v = glm::max<Type>(v, heights[ida+resolution]);
			v = glm::max<Type>(v, heights[ida+resolution+1]);
			heightsMipmap[0][idb] = v;
		}
	}

	for (int level = 1; level < levels; ++level) {
		size_t ida = 0;
		size_t idb = 0;
		size_t ra = resolution>>(level-1);
		size_t rb = resolution>>level;
		for (int y = 0; y < rb; ++y) {
			for (int x = 0; x < rb; ++x, ++idb, ida+=2) {
				Type v = heightsMipmap[level-1][ida];
				v = glm::max<Type>(v, heightsMipmap[level-1][ida+1]);
				v = glm::max<Type>(v, heightsMipmap[level-1][ida+ra]);
				v = glm::max<Type>(v, heightsMipmap[level-1][ida+ra+1]);
				heightsMipmap[level][idb] = v;
			}
			ida += ra;
		}
	}
}

template<bool SAFE>
void HeightMap_Header::SetNoMimmap(glm::ivec2 coord, Type value)
{
	heightsMipmap[0][Id<SAFE>(coord, 0)] = value;
}

bool HeightMap_Header::Update(glm::ivec2 coord, Type value)
{
	if (IsValidCoord(coord, -1) == false) {
		return false;
	}
	{
		heights[Id<false>(coord, -1)] = value;
		const size_t id = Id<false>(coord, -1);
		value = glm::max<Type>(value, heights[id]);
		value = glm::max<Type>(value, heights[id+1]);
		value = glm::max<Type>(value, heights[id+resolution]);
		value = glm::max<Type>(value, heights[id+resolution+1]);
		heightsMipmap[0][Id<false>(coord, 0)] = value;
	}
	for (int l=0; l<levels; ++l, coord >>= 1) {
		heightsMipmap[l][Id<false>(coord, l)] = value;
		coord = (coord>>1)<<1;
		const size_t id = Id<false>(coord, l);
		value = glm::max<Type>(value, heightsMipmap[l][id]);
		value = glm::max<Type>(value, heightsMipmap[l][id+1]);
		value = glm::max<Type>(value, heightsMipmap[l][id+(resolution>>l)]);
		value = glm::max<Type>(value, heightsMipmap[l][id+(resolution>>l)+1]);
	}
	return true;
}

template<bool SAFE>
HeightMap_Header::Type HeightMap_Header::Get(glm::ivec2 coord, int level) const
{
	return heightsMipmap[level][Id<true>(coord, -1)];
}

bool HeightMap_Header::SetMaterial(glm::ivec2 coord, MaterialType value)
{
	if (IsValidCoord(coord, -1) == false) {
		return false;
	}
	material[Id<true>(coord, -1)] = value;
	return true;
}

template<bool SAFE>
HeightMap_Header::MaterialType HeightMap_Header::GetMaterial(glm::ivec2 coord) const
{
	return material[Id<true>(coord, -1)];
}

template<bool SAFE>
size_t HeightMap_Header::Id(glm::ivec2 coord, int level) const
{
	if constexpr (SAFE) {
		coord = ClampCoord(coord, level);
	}
	if (level < 0) {
		return ((size_t)coord.x) + resolution * ((size_t)coord.y);
	}
	return ((size_t)coord.x) + (resMipmap>>level) * ((size_t)coord.y);
}

glm::ivec2 HeightMap_Header::ClampCoord(glm::ivec2 coord, int level) const
{
	return glm::clamp(coord, glm::ivec2{0, 0}, glm::ivec2(resolution>>level));
}

bool HeightMap_Header::IsValidCoord(glm::ivec2 coord, int level) const
{
	if (level < 0) {
		return coord.x >= 0 && coord.y >= 0 && coord.x < resMipmap && coord.y < resMipmap;
	}
	return coord.x >= 0 && coord.y >= 0 && coord.x < (resolution>>level) && coord.y < (resolution>>level);
}

spp::Aabb HeightMap_Header::GetAabb(const Transform &_trans) const
{
	glm::vec3 s = {horizontalSize, heightsMipmap[levels-1][0], horizontalSize};
	s *= 0.5f;
	Transform trans = _trans;
	trans.pos += s.y;
	return VertBox{s}.GetAabb(trans);
}


















bool HeightMap_Header::RayTest(const Transform &trans, const RayInfo &ray, float &near,
						glm::vec3 &normal) const
{
	return RayTestLocal(trans, ray, trans.ToLocal(ray), near, normal);
}

bool HeightMap_Header::RayTestLocal(const Transform &trans, const RayInfo &_ray,
							 const RayInfo &rayLocal, float &near,
							 glm::vec3 &normal) const
{
	RayInfo ray = rayLocal;
	
	ray.dir *= invScale;
	ray.invDir *= scale;
	ray.length *= invScale;
	ray.start.x += horizontalSize*0.5f;
	ray.start.z += horizontalSize*0.5f;
	ray.start *= invScale;
	ray.end.x += horizontalSize*0.5f;
	ray.end.z += horizontalSize*0.5f;
	ray.end *= invScale;
	
	assert(glm::distance(ray.end, ray.start + ray.dir) < 0.001f);
	
	near = 1.0f;

#define __COL_RTLN_RQ(DX, DZ)                                                  \
	RayTestLocalNode<false, false, DX, DZ>(rayLocal, near, normal, levels-1, 0, 0)
	if (!ray.signs[0]) {
		if (!ray.signs[2]) {
			if (!__COL_RTLN_RQ(true, true))
				return false;
		} else {
			if (!__COL_RTLN_RQ(true, false))
				return false;
		}
	} else {
		if (!ray.signs[2]) {
			if (!__COL_RTLN_RQ(false, true))
				return false;
		} else {
			if (!__COL_RTLN_RQ(false, false))
				return false;
		}
	}
#undef __COL_RTLN_RQ
	normal = trans.rot * glm::normalize(normal);
	return true;
}

template <bool TOP_ELSE_DOWN>
bool HeightMap_Header::TriangleRayTest(Type h00, Type hxy, Type h11, int x, int z,
								const RayInfo &localRay, float &near,
								glm::vec3 &localNormalUnnormalised) const
{
	assert(x >= 0 || z >= 0 || x + 1 < width || z + 1 < height);
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
	localNormalUnnormalised = n;
	return true;
}


template <bool BORDER_X, bool BORDER_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
bool HeightMap_Header::RayTestLocalNode(const RayInfo &rayLocal, float &near,
								 glm::vec3 &normal, int depth, int x,
								 int z) const
{
	assert(depth >= 0);
	assert(depth < levels);
	assert(x >= 0);
	assert(z >= 0);
	if constexpr (BORDER_X) {
		assert(x + 1 < mipmap[depth].width);
	}
	if constexpr (BORDER_Z) {
		assert(z + 1 < mipmap[depth].height);
	}
	if (depth == 0) {
		Type h00 = mipmap[0][x, z];
		Type h01 = mipmap[0][x, z + 1];
		Type h10 = mipmap[0][x + 1, z];
		Type h11 = mipmap[0][x + 1, z + 1];

		bool a, b;
		a = TriangleRayTest<false>(h00, h10, h11, x, z, rayLocal, near, normal);
		b = TriangleRayTest<true>(h00, h01, h11, x, z, rayLocal, near, normal);
		return a || b;
	} else {
		Aabb aabb{{x << depth, 0, z << depth},
				  {(x + 1) << depth, mipmap[depth][x, z], (z + 1) << depth}};
		float n, f;
		if (aabb.FastRayTest2(rayLocal.start, rayLocal.invDir, n, f) == false) {
			return false;
		}
		if (n > near) {
			return false;
		}
		x = x << 1;
		z = z << 1;
		depth = depth - 1;
#define __COL_RTLNCO(A, B)                                                     \
	RayTestLocalNodeCallOrdered<A, B, SIGN_DIR_X, SIGN_DIR_Z>(                 \
		rayLocal, near, normal, depth, x, z)
		if constexpr (BORDER_X) {
			if constexpr (BORDER_Z) {
				return __COL_RTLNCO(1, 1);
			} else {
				if (z + 1 < mipmap[depth].height) {
					return __COL_RTLNCO(1, 1);
				} else {
					return __COL_RTLNCO(1, 0);
				}
			}
		} else if constexpr (BORDER_Z) {
			if (x + 1 < mipmap[depth].width) {
				return __COL_RTLNCO(1, 1);
			} else {
				return __COL_RTLNCO(0, 1);
			}
		} else {
			if (x + 1 < mipmap[depth].width) {
				if (z + 1 < mipmap[depth].height) {
					return __COL_RTLNCO(1, 1);
				} else {
					return __COL_RTLNCO(1, 0);
				}
			} else {
				if (z + 1 < mipmap[depth].height) {
					return __COL_RTLNCO(0, 1);
				} else {
					return __COL_RTLNCO(0, 0);
				}
			}
		}
	}
#undef __COL_RTLNCO
}

bool HeightMap_Header::CylinderTestOnGround(const Transform &trans,
									 const Cylinder &cyl, glm::vec3 pos,
									 float &offsetHeight) const
{
	pos = trans.ToLocal(pos) * invScale;
	int x = pos.x;
	int y = pos.y;
	float fracx = pos.x - x;
	float fracy = pos.y - y;
	if (x >= 0 || y >= 0 || x + 1 < width || y + 1 < height) {
		return false;
	}

	Type a00 = mipmap[0][x, y];
	Type a11 = mipmap[0][x + 1, y + 1];

	if (glm::abs(a00 - a11) > maxDh11) {
		return false;
	}

	float hdiag = a00 * (1.0f - fracy) + a11 * fracy;

	if (fracy > fracx) { // upper triangle
		float a01 = mipmap[0][x, y + 1];
		float f = fracx / fracy;

		if (glm::abs(a00 - a01) > maxDh1 || glm::abs(a11 - a01) > maxDh1) {
			return false;
		}

		float hy = a00 * (1.0f - fracy) + a01 * fracy;
		float h = hy * (1.0f - f) + hdiag * f;

		offsetHeight = trans.pos.y - h;
		return true;
	} else { // lower triangle
		float f = (fracx - fracy) / (1.0f - fracy);
		float a10 = mipmap[0][x + 1, y];

		if (glm::abs(a00 - a10) > maxDh1 || glm::abs(a11 - a10) > maxDh1) {
			return false;
		}

		if (fracx == 0.0f) {
			offsetHeight = trans.pos.y - a00;
			return true;
		}

		float hy = a10 * (1.0f - fracy) + a11 * fracy;
		float h = hy * f + hdiag * (1.0f - f);

		if (glm::abs(a00 - a10) > maxDh1 || glm::abs(a11 - a10) > maxDh1) {
			return false;
		}

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

template void HeightMap_Header::SetNoMimmap<true>(glm::ivec2 coord, Type value);
template void HeightMap_Header::SetNoMimmap<false>(glm::ivec2 coord,
												   Type value);
template HeightMap_Header::Type HeightMap_Header::Get<true>(glm::ivec2 coord,
															int level) const;
template HeightMap_Header::Type HeightMap_Header::Get<false>(glm::ivec2 coord,
															 int level) const;
template HeightMap_Header::MaterialType
HeightMap_Header::GetMaterial<true>(glm::ivec2 coord) const;
template HeightMap_Header::MaterialType
HeightMap_Header::GetMaterial<false>(glm::ivec2 coord) const;
template size_t HeightMap_Header::Id<true>(glm::ivec2 coord, int level) const;
template size_t HeightMap_Header::Id<false>(glm::ivec2 coord, int level) const;
} // namespace Collision3D
