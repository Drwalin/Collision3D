// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "CollisionAlgorithms.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{
// Origin at center bottom of AABB
struct HeightMap {
	using Type = float;
	using MaterialType = uint8_t;

	HeightMap();
	~HeightMap();

	HeightMap(HeightMap &other);
	HeightMap(HeightMap &&other);
	HeightMap(const HeightMap &other);

	HeightMap &operator=(HeightMap &other);
	HeightMap &operator=(HeightMap &&other);
	HeightMap &operator=(const HeightMap &other);

	void Init(int width, int height);

	// Treating cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()

	void Update(glm::ivec2 coord, Type value);
	Type Get(int level, glm::ivec2 coord) const;

	void SetMaterial(glm::ivec2 coord, MaterialType value);
	MaterialType GetMaterial(int level, glm::ivec2 coord) const;

	glm::ivec2 ConvertGlobalPosToCoord(const Transform &trans,
									   glm::vec3 pos) const;

public:
	struct Header {
		size_t bytes;

		float minimum = 1e9;
		int levels = 0;

		glm::vec3 size;
		glm::vec3 halfSize;
		glm::vec3 scale;	// .x === .z
		glm::vec3 invScale; // 1 / scale

		// should mean >~ 45 degree for normal for walking on ground
		Type maxDh1;  // height difference along axes
		Type maxDh11; // height difference along diagonal

		glm::ivec2 sizes[16];
		// diagonal is between (x, y) and (x+1, y+1)
		Type *heightsMipmap[16];
		MaterialType *material;

	public:
		static Header *Allocate(int width, int height);

	public:
		glm::ivec2 ConvertGlobalPosToCoord(const Transform &trans,
										   glm::vec3 pos) const;

		void Set(const glm::vec3 &scale, const glm::vec3 &size, Type *heights,
				 MaterialType *materials);

		void InitValues(const glm::vec3 &scale, const glm::vec3 &size);
		void SetNoMimmap(glm::ivec2 coord, Type value);
		void GenerateMipmap();

		void Update(glm::ivec2 coord, Type value);
		Type Get(int level, glm::ivec2 coord) const;

		void SetMaterial(glm::ivec2 coord, MaterialType value);
		MaterialType GetMaterial(int level, glm::ivec2 coord) const;

		template <bool TOP_ELSE_DOWN>
		bool TriangleRayTest(Type h00, Type hxy, Type h11, int x, int z,
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

private:
	void CopyIntoThis(const HeightMap &src);
};
} // namespace Collision3D
