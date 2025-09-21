// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <vector>
#include <memory>

#include "HeightMapUtil.hpp"
#include "CollisionAlgorithms.hpp"

namespace Collision3D
{
// Origin at center of base
// Can be both wall and floor
struct VertBox {
	glm::vec3 halfExtents;

	// Collision treats cylinder as aligned square prism to tran
	COLLISION_SHAPE_METHODS_DECLARATION()
	void CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
											   const Cylinder &cyl,
											   glm::vec3 pos,
											   float &offsetHeight) const;
};

// Origin at center of base
// Can be both wall and floor
struct Cylinder {
	float height;
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
	void CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
											   const Cylinder &cyl,
											   glm::vec3 pos,
											   float &offsetHeight) const;
};

// Origin at center
// Ray only
struct Sphere {
	float radius;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge (-x, +x), and extends
struct RampRectangle {
	float halfWidth;	 // expands (-x/2 ; +x/2)
	float height;		 // expands +y
	float depth;		 // expands +z
	float halfThickness; // thickens symmetrically y

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center of first horizontal edge (-x, +x)
struct RampTriangle {
	float sideLength;
	float heightOfVertexOnZ; // height of vertex at (0, Y, +z)

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct Triangle {
	glm::vec3 a, b, c;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at p0
// Normal without roatation: z=1
struct VerticalTriangle {
	glm::vec2 p1, p2;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

// Origin at center bottom of AABB
template <typename T, typename MT> struct HeightMap {
	glm::vec3 size;
	glm::vec3 halfSize;
	glm::vec3 scale;	// .x === .z
	glm::vec3 invScale; // 1 / scale
	int width;
	int height;

	// should mean > 46 degree
	T maxDh1;
	T maxDh11;

	// diagonal is between (x, y) and (x+1, y+1)

	std::vector<Matrix<T>> mipmap;
	Matrix<MT> material;

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
						  glm::vec3 &localNormalUnnormalised, int depth, int x,
						  int z) const;
	template <bool SAFE_X, bool SAFE_Z, bool SIGN_DIR_X, bool SIGN_DIR_Z>
	bool RayTestLocalNodeCallOrdered(const RayInfo &rayLocal, float &near,
									 glm::vec3 &localNormalUnnormalised,
									 int depth, int x, int z) const;

	HeightMap() : width(0), height(0) {}

	HeightMap(HeightMap &other) = default;
	HeightMap(HeightMap &&other) = default;
	HeightMap(const HeightMap &other) = default;

	HeightMap &operator=(HeightMap &other) = default;
	HeightMap &operator=(HeightMap &&other) = default;
	HeightMap &operator=(const HeightMap &other) = default;

	// Treting cylinder as point at it's origin
	COLLISION_SHAPE_METHODS_DECLARATION()
};
extern template struct HeightMap<int8_t, uint8_t>;
extern template struct HeightMap<uint8_t, uint8_t>;
extern template struct HeightMap<int16_t, uint8_t>;
extern template struct HeightMap<uint16_t, uint8_t>;
extern template struct HeightMap<int32_t, uint8_t>;
extern template struct HeightMap<uint32_t, uint8_t>;
extern template struct HeightMap<float, uint8_t>;

// Origin at base
struct VerticalCappedCone {
	float baseRadius;
	float topRadius;
	float height;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct AnyPrimitive {
	union {
		VertBox vertBox;
		Cylinder cylinder;
		Sphere sphere;
		RampRectangle rampRectangle;
		VerticalTriangle vertTriangle;
		VerticalCappedCone cappedCone;
		RampTriangle rampTriangle;
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		VERTBOX = 1,
		CYLINDER = 2,
		SPHERE = 3,
		RAMP_RECTANGLE = 4,
		VERTICAL_TRIANGLE = 5,
		CAPPED_CONE = 6,
		RAMP_TRIANGLE = 7,
	} type = INVALID;

	AnyPrimitive(VertBox vertBox, Transform trans = {});
	AnyPrimitive(Cylinder cylinder, Transform trans = {});
	AnyPrimitive(Sphere sphere, Transform trans = {});
	AnyPrimitive(RampRectangle rampRectangle, Transform trans = {});
	AnyPrimitive(VerticalTriangle vertTriangle, Transform trans = {});
	AnyPrimitive(VerticalCappedCone cappedCone, Transform trans = {});
	AnyPrimitive(RampTriangle rampTriangle, Transform trans = {});

	AnyPrimitive &operator=(VertBox vertBox);
	AnyPrimitive &operator=(Cylinder cylinder);
	AnyPrimitive &operator=(Sphere sphere);
	AnyPrimitive &operator=(RampRectangle rampRectangle);
	AnyPrimitive &operator=(VerticalTriangle vertTriangle);
	AnyPrimitive &operator=(VerticalCappedCone cappedCone);
	AnyPrimitive &operator=(RampTriangle rampTriangle);

	AnyPrimitive() = default;

	AnyPrimitive(AnyPrimitive &other) = default;
	AnyPrimitive(AnyPrimitive &&other) = default;
	AnyPrimitive(const AnyPrimitive &other) = default;

	AnyPrimitive &operator=(AnyPrimitive &other) = default;
	AnyPrimitive &operator=(AnyPrimitive &&other) = default;
	AnyPrimitive &operator=(const AnyPrimitive &other) = default;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct CompoundPrimitive {
	std::vector<AnyPrimitive> primitives;

	CompoundPrimitive() = default;

	CompoundPrimitive(CompoundPrimitive &other) = default;
	CompoundPrimitive(CompoundPrimitive &&other) = default;
	CompoundPrimitive(const CompoundPrimitive &other) = default;

	CompoundPrimitive &operator=(CompoundPrimitive &other) = default;
	CompoundPrimitive &operator=(CompoundPrimitive &&other) = default;
	CompoundPrimitive &operator=(const CompoundPrimitive &other) = default;

	COLLISION_SHAPE_METHODS_DECLARATION()
};

struct AnyShape {
	union {
		VertBox vertBox;
		Cylinder cylinder;
		Sphere sphere;
		RampRectangle rampRectangle;
		VerticalTriangle vertTriangle;
		VerticalCappedCone cappedCone;
		RampTriangle rampTriangle;
		std::unique_ptr<HeightMap<float, uint8_t>> heightMap;
		CompoundPrimitive compound;
	};

	Transform trans;

	enum Type : uint8_t {
		INVALID = 0,
		VERTBOX = 1,
		CYLINDER = 2,
		SPHERE = 3,
		RAMP_RECTANGLE = 4,
		VERTICAL_TRIANGLE = 5,
		CAPPED_CONE = 6,
		RAMP_TRIANGLE = 7,
		HEIGHT_MAP = 62,
		COMPOUND = 63,
	} type = INVALID;

	AnyShape(VertBox vertBox, Transform trans = {});
	AnyShape(Cylinder cylinder, Transform trans = {});
	AnyShape(Sphere sphere, Transform trans = {});
	AnyShape(RampRectangle rampRectangle, Transform trans = {});
	AnyShape(VerticalTriangle vertTriangle, Transform trans = {});
	AnyShape(VerticalCappedCone cappedCone, Transform trans = {});
	AnyShape(RampTriangle rampTriangle, Transform trans = {});
	AnyShape(HeightMap<float, uint8_t> &&heightMap, Transform trans = {});
	AnyShape(CompoundPrimitive &&compound, Transform trans = {});

	AnyShape &operator=(VertBox vertBox);
	AnyShape &operator=(Cylinder cylinder);
	AnyShape &operator=(Sphere sphere);
	AnyShape &operator=(RampRectangle rampRectangle);
	AnyShape &operator=(VerticalTriangle vertTriangle);
	AnyShape &operator=(VerticalCappedCone cappedCone);
	AnyShape &operator=(RampTriangle rampTriangle);
	AnyShape &operator=(HeightMap<float, uint8_t> &&heightMap);
	AnyShape &operator=(CompoundPrimitive &&compound);

	AnyShape();

	AnyShape(AnyShape &other);
	AnyShape(AnyShape &&other);
	AnyShape(const AnyShape &other);

	AnyShape &operator=(AnyShape &other);
	AnyShape &operator=(AnyShape &&other);
	AnyShape &operator=(const AnyShape &other);

	~AnyShape();

	COLLISION_SHAPE_METHODS_DECLARATION()
};
} // namespace Collision3D
