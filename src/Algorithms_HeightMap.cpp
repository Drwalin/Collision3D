// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstring>
#include <cstdlib>

#include "../include/collision3d/CollisionShapes_HeightMapHeader.hpp"
#include "../include/collision3d/CollisionShapes_HeightMap.hpp"

namespace Collision3D
{
using namespace spp;

HeightMap::HeightMap() : header(nullptr) {}

HeightMap::~HeightMap()
{
	if (header) {
		free(header);
		header = nullptr;
	}
}

HeightMap::HeightMap(HeightMap &other)
{
	assert(!"Shouldn't be used");
	header = nullptr;
	CopyIntoThis(other);
}

HeightMap::HeightMap(HeightMap &&other) : header(other.header)
{
	other.header = nullptr;
}

HeightMap::HeightMap(const HeightMap &other)
{
	assert(!"Shouldn't be used");
	header = nullptr;
	CopyIntoThis(other);
}

HeightMap &HeightMap::operator=(HeightMap &other)
{
	assert(!"Shouldn't be used");
	CopyIntoThis(other);
}

HeightMap &HeightMap::operator=(HeightMap &&other)
{
	header = other.header;
	other.header = nullptr;
	return *this;
}

HeightMap &HeightMap::operator=(const HeightMap &other)
{
	assert(!"Shouldn't be used");
	CopyIntoThis(other);
}

void HeightMap::CopyIntoThis(const HeightMap &other)
{
	assert(!"Shouldn't be used");
	this->~HeightMap();

	if (other.header) {
		header = (HeightMap_Header *)malloc(other.header->bytes);
		memcpy(header, other.header, other.header->bytes);
		size_t diff = ((size_t)header) - ((size_t)other.header);
		*((size_t *)&(header->material)) += diff;
		*((size_t *)&(header->heights)) += diff;
	} else {
		header = nullptr;
	}
}

void HeightMap::Init(glm::ivec2 resolution)
{
	this->~HeightMap();
	header = HeightMap_Header::Allocate(resolution);
}

void HeightMap::InitMeta(float horizontalScale, float verticalScale)
{
	assert(header);
	header->InitMeta(horizontalScale, verticalScale);
}

spp::Aabb HeightMap::GetAabb(const Transform &trans) const
{
	assert(header);
	return header->GetAabb(trans);
}

bool HeightMap::RayTest(const Transform &trans, const RayInfo &ray, float &near,
						glm::vec3 &normal) const
{
	assert(header);
	return header->RayTest(trans, ray, near, normal);
}

bool HeightMap::RayTestLocal(const RayInfo &ray, float &near,
							 glm::vec3 &normal) const
{
	assert(header);
	return header->RayTestLocal(ray, near, normal);
}

bool HeightMap::CylinderTestOnGround(const Transform &trans,
									 const Cylinder &cyl, glm::vec3 pos,
									 float &offsetHeight,
									 glm::vec3 *onGroundNormal) const
{
	assert(header);
	return header->CylinderTestOnGround(trans, cyl, pos, offsetHeight,
										onGroundNormal);
}

bool HeightMap::CylinderTestMovement(const Transform &trans,
									 float &validMovementFactor,
									 const Cylinder &cyl,
									 const RayInfo &movementRay,
									 glm::vec3 &normal) const
{
	assert(header);
	return header->CylinderTestMovement(trans, validMovementFactor, cyl,
										movementRay, normal);
}

bool HeightMap::Update(glm::ivec2 coord, Type value)
{
	assert(header);
	return header->Update(coord, value);
}

HeightMap::Type HeightMap::Get(glm::ivec2 coord) const
{
	assert(header);
	return header->Get<true>(coord);
}

bool HeightMap::SetMaterial(glm::ivec2 coord, MaterialType value)
{
	assert(header);
	return header->SetMaterial(coord, value);
}

HeightMap::MaterialType HeightMap::GetMaterial(glm::ivec2 coord) const
{
	assert(header);
	return header->GetMaterial<true>(coord);
}

glm::ivec2 HeightMap::ConvertGlobalPosToCoord(const Transform &trans,
											  glm::vec3 pos) const
{
	assert(header);
	return header->ConvertGlobalPosToCoord(trans, pos);
}

const HeightMap::Type *HeightMap::GetHeights() const
{
	assert(header);
	return header->heights;
}

HeightMap::Type *HeightMap::AccessHeights()
{
	assert(header);
	return header->heights;
}

const HeightMap::MaterialType *HeightMap::GetMaterial() const
{
	assert(header);
	return header->material;
}

HeightMap::MaterialType *HeightMap::AccessMaterial()
{
	assert(header);
	return header->material;
}

bool HeightMap::IsValid() const { return header; }
} // namespace Collision3D
