// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "HeightMapUtil.hpp"
#include "CollisionAlgorithms.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{
// Origin at center bottom of AABB
struct HeightMap {
	using T = float;
	using MT = uint8_t;

	HeightMap() : header(nullptr) {}

	HeightMap(HeightMap &other) = default;
	HeightMap(HeightMap &&other) = default;
	HeightMap(const HeightMap &other) = default;

	HeightMap &operator=(HeightMap &other) = default;
	HeightMap &operator=(HeightMap &&other) = default;
	HeightMap &operator=(const HeightMap &other) = default;
	
	void Init(int width, int height);

	// Treating cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()

private:
	struct Header {
		const int width;
		const int height;

		glm::vec3 size;
		glm::vec3 halfSize;
		glm::vec3 scale;	// .x === .z
		glm::vec3 invScale; // 1 / scale

		// should mean > 46 degree
		T maxDh1;
		T maxDh11;

		// diagonal is between (x, y) and (x+1, y+1)
		T *heightsMipmap[16];
		T *heights;
		MT *material;
		
	public:
		
		static Header *Allocate(int width, int height);

	public:
		
		glm::ivec2 GetClosestPointIfAny(glm::vec2 verticalPos);

		void InitSet(int width, int height, const glm::vec3 &scale,
					 const glm::vec3 &size, T *heights, MT *materials);

		void InitValues(int width, int height, const glm::vec3 &scale,
						const glm::vec3 &size);
		void GenerateMipmap();
		void Update(int x, int y, T value);
		T GetMax2x2(const Matrix<T> &mat, int x, int y) const;

		template <bool TOP_ELSE_DOWN>
		bool TriangleRayTest(T h00, T hxy, T h11, int x, int z,
							 const RayInfo &localRay, float &near,
							 glm::vec3 &localNormalUnnormalised) const;

		template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
		bool RayTestLocalNode(const RayInfo &rayLocal, float &near,
							  glm::vec3 &localNormalUnnormalised, int depth,
							  int x, int z) const;
		template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
		bool RayTestLocalNodeCallOrdered(const RayInfo &rayLocal, float &near,
										 glm::vec3 &localNormalUnnormalised,
										 int depth, int x, int z) const;

		COLLISION_SHAPE_METHODS_DECLARATION()
	};

	Header *header = nullptr;
};
} // namespace Collision3D
