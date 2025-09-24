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

	glm::vec2 size;

	glm::vec3 scale;
	glm::vec3 invScale;

	static constexpr float MAX_HEIGHT = 100000;

	// should mean >~ 45 degree for normal for walking on ground
	Type maxDh1;  // height difference along axes
	Type maxDh11; // height difference along diagonal

	glm::ivec2 resolution;
	// diagonal is between (x, y) and (x+1, y+1)
	Type *heights;
	MaterialType *material;

public:
	static HeightMap_Header *Allocate(glm::ivec2 resolution);

public:
	glm::ivec2 ConvertGlobalPosToCoord(const Transform &trans,
									   glm::vec2 pos2d) const;
	glm::vec3 ConvertToLocalPos(const Transform &trans, glm::vec3 pos) const;

	void InitMeta(float horizontalScale, float verticalScale);

	bool Update(glm::ivec2 coord, Type value);
	template <bool SAFE> Type Get(glm::ivec2 coord) const;

	bool SetMaterial(glm::ivec2 coord, MaterialType value);
	template <bool SAFE> MaterialType GetMaterial(glm::ivec2 coord) const;

	COLLISION_SHAPE_METHODS_DECLARATION()

private:
	template <bool SIGN_POSITIVE_DIR_X, bool SIGN_POSITIVE_DIR_Z>
	bool RayTestGrid(const RayInfo &ray, float &near, glm::vec3 &normal) const;

	bool RayTestCell(const RayInfo &ray, float &near, glm::vec3 &normal, int x,
					 int z, float t, float nextt) const;

	template <bool TOP_ELSE_DOWN>
	bool TriangleRayTest(Type h00, Type hxy, Type h11, int x, int z,
						 const RayInfo &localRay, float &near,
						 glm::vec3 &normal) const;

private:
	bool IsValidCoord(glm::ivec2 coord) const;
	glm::ivec2 ClampCoord(glm::ivec2 coord) const;
	template <bool SAFE> size_t Id(glm::ivec2 coord) const;
};
} // namespace Collision3D
