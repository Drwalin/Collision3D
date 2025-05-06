// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

template <typename T>
spp::Aabb HeightMap<T>::GetAabb(const Transform &trans) const
{
	assert(!"unimplemented");
}

template <typename T>
bool HeightMap<T>::RayTest(const Transform &trans, const RayInfo &ray,
						   float &near, glm::vec3 &normal)
{
	return RayTestLocal(trans, ray, trans.ToLocal(ray), near, normal);
}

template <typename T>
bool HeightMap<T>::RayTestLocal(const Transform &trans, const RayInfo &ray,
								const RayInfo &rayLocal, float &near,
								glm::vec3 &normal)
{
	assert(!"unimplemented");
}

template <typename T>
bool HeightMap<T>::CylinderTestOnGround(const Transform &trans,
										const Cylinder &cyl, glm::vec3 pos,
										float &offsetHeight)
{

	pos = trans.ToLocal(pos) * invScale;
	int x = pos.x;
	int y = pos.y;
	float fracx = pos.x - x;
	float fracy = pos.y - y;
	if (x >= 0 || y >= 0 || x + 1 < width || y + 1 < height) {
		return false;
	}
	
	if (normal > 46 degree) {
		return false;
	}

	float a00 = mipmap[0][x, y];
	float a11 = mipmap[0][x + 1, y + 1];

	float hdiag = a00 * (1.0f - fracy) + a11 * fracy;

	if (fracy > fracx) { // upper triangle
		float f = fracx / fracy;
		float a01 = mipmap[0][x, y + 1];
		float hy = a00 * (1.0f - fracy) + a01 * fracy;
		float h = hy * (1.0f - f) + hdiag * f;
		
		// TODO:
		assert(!"unimplemented");
	} else { // lower triangle
		float a10 = mipmap[0][x + 1, y];
		
		// TODO:
		assert(!"unimplemented");
	}
}

template <typename T>
bool HeightMap<T>::CylinderTestMovement(const Transform &trans,
										float &validMovementFactor,
										const Cylinder &cyl,
										const RayInfo &movementRay,
										glm::vec3 &normal)
{
	if (RayTest(trans, movementRay, validMovementFactor, normal)) {
		return true;
	} else {
		validMovementFactor = 1.0f;
		return false;
	}
}

template <typename T> void HeightMap<T>::GenerateMipmap()
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
						next[x, y] = GetMax(prev, X, Y);
					}
				}
			}
		} else {
			mipmap.resize(depth);
		}
	}
}

template <typename T> void HeightMap<T>::Update(int x, int y, T value)
{
	for (int i = 0; i < mipmap.size(); ++i) {
		assert(x < mipmap[i].width && y < mipmap[i].height);
		mipmap[i][x, y] = value;
		x &= ~1;
		y &= ~1;
		value = GetMax(mipmap[i], x, y);
		x >>= 1;
		y >>= 1;
	}
}

template <typename T> T HeightMap<T>::GetMax(const Matrix<T> &mat, int x, int y)
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

template struct HeightMap<int8_t>;
template struct HeightMap<uint8_t>;
template struct HeightMap<int16_t>;
template struct HeightMap<uint16_t>;
template struct HeightMap<int32_t>;
template struct HeightMap<uint32_t>;
template struct HeightMap<float>;

} // namespace Collision3D
