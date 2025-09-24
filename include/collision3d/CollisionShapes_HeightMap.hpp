// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "CollisionAlgorithms.hpp"
#include "ForwardDeclarations.hpp"

namespace Collision3D
{
// Origin at center bottom of AABB
// Assuming there is no lower points than Y=0
// HeightMap needs to be square with side length 2^X+1
struct HeightMap {
	using Type = HeightMap_Type;
	using MaterialType = HeightMap_MaterialType;

	HeightMap();
	~HeightMap();

	HeightMap(HeightMap &other);
	HeightMap(HeightMap &&other);
	HeightMap(const HeightMap &other);

	HeightMap &operator=(HeightMap &other);
	HeightMap &operator=(HeightMap &&other);
	HeightMap &operator=(const HeightMap &other);

	void Init(int resolution);

	// Treating cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()

	bool Update(glm::ivec2 coord, Type value);
	Type Get(glm::ivec2 coord, int level) const;
	
	const Type *GetHeights() const;
	Type *AccessHeights();
	
	const MaterialType *GetMaterial() const;
	MaterialType *AccessMaterial();

	bool SetMaterial(glm::ivec2 coord, MaterialType value);
	MaterialType GetMaterial(glm::ivec2 coord) const;

	glm::ivec2 ConvertGlobalPosToCoord(const Transform &trans,
									   glm::vec3 pos) const;
	
	bool IsValid() const;

public:
	HeightMap_Header *header = nullptr;

private:
	void CopyIntoThis(const HeightMap &src);
};
} // namespace Collision3D
