// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

void HeightMap::InitSet(int width, int height, const glm::vec3 &scale,
						const glm::vec3 &size, T *heights, MT *materials)
{
	InitValues(width, height, scale, size);
	material.heights.clear();
	material.heights.insert(material.heights.begin(), materials,
							materials + (width * height));
	GenerateMipmap();
}

void HeightMap::InitValues(int width, int height, const glm::vec3 &scale,
						   const glm::vec3 &size)
{
	halfSize = size / 2.0f;
	invScale = scale;
	mipmap.resize(1);
	mipmap[0].width = width;
	mipmap[0].height = height;
	material.height = height;
	material.width = width;
}

template <bool TOP_ELSE_DOWN>
bool HeightMap::TriangleRayTest(T h00, T hxy, T h11, int x, int z,
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

spp::Aabb HeightMap::GetAabb(const Transform &trans) const
{
	return VertBox{halfSize}.GetAabb(trans);
}

bool HeightMap::RayTest(const Transform &trans, const RayInfo &ray, float &near,
						glm::vec3 &normal) const
{
	return RayTestLocal(trans, ray, trans.ToLocal(ray), near, normal);
}

bool HeightMap::RayTestLocal(const Transform &trans, const RayInfo &ray,
							 const RayInfo &rayLocal, float &near,
							 glm::vec3 &normal) const
{
	near = 1.0f;
	const int depth = mipmap.size() - 1;
	const glm::vec3 d = rayLocal.dir;

	int xdir = d.x >= 0.0f ? 1 : -1;
	int xstart = d.x >= 0.0f ? 0 : mipmap.back().width - 1;
	int xend = d.x >= 0.0f ? mipmap.back().width : -1;
	int zdir = d.z >= 0.0f ? 1 : -1;
	int zstart = d.z >= 0.0f ? 0 : mipmap.back().height - 1;
	int zend = d.z >= 0.0f ? mipmap.back().height : -1;

#define __COL_RTLN_RQ(DX, DZ)                                                  \
	RayTestLocalNode<false, false, DX, DZ>(rayLocal, near, normal, depth, x, z)
	for (int x = xstart; x != xend; x += xdir) {
		for (int z = zstart; z != zend; z += zdir) {
			if (d.x > 0) {
				if (d.z > 0) {
					if (!__COL_RTLN_RQ(true, true))
						return false;
				} else {
					if (!__COL_RTLN_RQ(true, false))
						return false;
				}
			} else if (d.z > 0) {
				if (!__COL_RTLN_RQ(false, true))
					return false;
			} else {
				if (!__COL_RTLN_RQ(false, false))
					return false;
			}
		}
	}
#undef __COL_RTLN_RQ
	normal = trans * glm::normalize(normal);
	return true;
}

template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
bool HeightMap::RayTestLocalNode(const RayInfo &rayLocal, float &near,
								 glm::vec3 &normal, int depth, int x,
								 int z) const
{
	assert(depth >= 0);
	assert(depth < mipmap.size());
	assert(x >= 0);
	assert(z >= 0);
	if constexpr (SAFE_X) {
		assert(x + 1 < mipmap[depth].width);
	}
	if constexpr (SAFE_Z) {
		assert(z + 1 < mipmap[depth].height);
	}
	if (depth == 0) {
		T h00 = mipmap[0][x, z];
		T h01 = mipmap[0][x, z + 1];
		T h10 = mipmap[0][x + 1, z];
		T h11 = mipmap[0][x + 1, z + 1];

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
		if constexpr (SAFE_X) {
			if constexpr (SAFE_Z) {
				return __COL_RTLNCO(1, 1);
			} else {
				if (z + 1 < mipmap[depth].height) {
					return __COL_RTLNCO(1, 1);
				} else {
					return __COL_RTLNCO(1, 0);
				}
			}
		} else if constexpr (SAFE_Z) {
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

template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
bool HeightMap::RayTestLocalNodeCallOrdered(const RayInfo &rayLocal,
											float &near, glm::vec3 &normal,
											int depth, int x, int z) const
{
	bool res = false;
#define __COL_RTLN(DX, DZ)                                                     \
	RayTestLocalNode<true, true, SIGN_DIR_X, SIGN_DIR_Z>(                      \
		rayLocal, near, normal, depth, x + DX, z + DZ)
	if constexpr (SIGN_DIR_X) {
		if constexpr (SIGN_DIR_Z) {
			res |= __COL_RTLN(0, 0);
			res |= __COL_RTLN(0, 1);
			res |= __COL_RTLN(1, 0);
			res |= __COL_RTLN(1, 1);
		} else {
			res |= __COL_RTLN(0, 1);
			res |= __COL_RTLN(0, 0);
			res |= __COL_RTLN(1, 1);
			res |= __COL_RTLN(1, 0);
		}
	} else if constexpr (SIGN_DIR_Z) {
		res |= __COL_RTLN(1, 0);
		res |= __COL_RTLN(1, 1);
		res |= __COL_RTLN(0, 0);
		res |= __COL_RTLN(0, 1);
	} else {
		res |= __COL_RTLN(1, 1);
		res |= __COL_RTLN(1, 0);
		res |= __COL_RTLN(0, 1);
		res |= __COL_RTLN(0, 0);
	}
#undef __COL_RTLN
	return res;
}

bool HeightMap::CylinderTestOnGround(const Transform &trans,
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

	T a00 = mipmap[0][x, y];
	T a11 = mipmap[0][x + 1, y + 1];

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

bool HeightMap::CylinderTestMovement(const Transform &trans,
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

void HeightMap::GenerateMipmap()
{
	for (int depth = 1;; ++depth) {
		const auto &prev = mipmap[depth - 1];
		if (prev.height >= 2 && prev.width >= 2) {
			if (depth >= mipmap.size()) {
				mipmap.push_back({(prev.width + 1) / 2, (prev.height + 1) / 2});
			}
			auto &next = mipmap[depth];
			for (int y = 0, Y = 0; y < next.height; ++y, Y += 2) {
				for (int x = 0, X = 0; x < next.width; ++x, X += 2) {
					if (x + 1 < next.width && y + 1 < next.height) {
						const int X1 = X + 1;
						const int Y1 = Y + 1;
						next[x, y] =
							glm::max(glm::max(glm::max(prev[X, Y], prev[X, Y1]),
											  prev[X1, Y]),
									 prev[X1, Y1]);
					} else {
						next[x, y] = GetMax2x2(prev, X, Y);
					}
				}
			}
		} else {
			mipmap.resize(depth);
		}
	}
}

void HeightMap::Update(int x, int y, T value)
{
	for (int i = 0; i < mipmap.size(); ++i) {
		assert(x < mipmap[i].width && y < mipmap[i].height);
		mipmap[i][x, y] = value;
		x &= ~1;
		y &= ~1;
		value = GetMax2x2(mipmap[i], x, y);
		x >>= 1;
		y >>= 1;
	}
}

HeightMap::T HeightMap::GetMax2x2(const Matrix<T> &mat, int x, int y) const
{
	const int x1 = x + 1;
	const int y1 = y + 1;
	T v = mat[x, y];
	if (y1 < mat.height) {
		if (x1 < mat.width) {
			return glm::max(glm::max(glm::max(v, mat[x, y1]), mat[x1, y]),
							mat[x1, y1]);
		} else {
			return glm::max(v, mat[x, y1]);
		}
	} else {
		if (x1 < mat.width) {
			return glm::max(v, mat[x1, y]);
		} else {
		}
	}
	return v;
}
} // namespace Collision3D
