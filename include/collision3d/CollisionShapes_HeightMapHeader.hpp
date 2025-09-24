// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "CollisionAlgorithms.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{
struct HeightMap_Header {
	using Type = HeightMap_Type;
	using MaterialType = HeightMap_MaterialType;
	
	size_t bytes;

	int levels = 0;

	float horizontalSize;
	float scale;
	float invScale; // 1 / scale

	// should mean >~ 45 degree for normal for walking on ground
	Type maxDh1;  // height difference along axes
	Type maxDh11; // height difference along diagonal

	int resolution;
	int resMipmap;
	// diagonal is between (x, y) and (x+1, y+1)
	Type *heights;
	Type *heightsMipmap[16];
	MaterialType *material;

public:
	static HeightMap_Header *Allocate(int resolution);

public:
	glm::ivec2 ConvertGlobalPosToCoord(const Transform &trans,
									   glm::vec3 pos) const;

	void InitMeta(float scale);
	void GenerateMipmap();
	
	template<bool SAFE>
	void SetNoMimmap(glm::ivec2 coord, Type value);

	bool Update(glm::ivec2 coord, Type value);
	template<bool SAFE>
	Type Get(glm::ivec2 coord, int level) const;

	bool SetMaterial(glm::ivec2 coord, MaterialType value);
	template<bool SAFE>
	MaterialType GetMaterial(glm::ivec2 coord) const;
	
	
	
	
	
	
	
	

	template <bool TOP_ELSE_DOWN>
	bool TriangleRayTest(Type h00, Type hxy, Type h11, int x, int z,
						 const RayInfo &localRay, float &near,
						 glm::vec3 &localNormalUnnormalised) const;
	
	bool RayTastSquare(const RayInfo &ray, float &near, int x, int z,
					   glm::vec3 &normal, Type heights[2][2]) const;

	template <bool SIGN_DIR_X, bool SIGN_DIR_Z>
	bool RayTestGrid(const RayInfo &rayLocal, float &near,
						  glm::vec3 &normal) const;

	COLLISION_SHAPE_METHODS_DECLARATION()

private:
	bool IsValidCoord(glm::ivec2 coord, int level) const;
	glm::ivec2 ClampCoord(glm::ivec2 coord, int level) const;
	template<bool SAFE>
	size_t Id(glm::ivec2 coord, int level) const;
};
} // namespace Collision3D
