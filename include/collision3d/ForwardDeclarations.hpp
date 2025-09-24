// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

namespace Collision3D
{
struct VertBox;
struct Cylinder;
struct Sphere;
struct RampRectangle;
struct HeightMap;
struct HeightMap_Header;

struct CompoundPrimitive;
struct AnyShape;
struct AnyPrimitive;

struct Rotation;
struct Transform;

using HeightMap_Type = float;
using HeightMap_MaterialType = uint8_t;
} // namespace Collision3D
