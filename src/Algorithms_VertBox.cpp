// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/collision3d/CollisionShapes.hpp"

namespace Collision3D
{
using namespace spp;

spp::Aabb VertBox::GetAabb(const Transform &trans) const
{
	Rotation rot = trans.rot;
	if (rot.value >= 120) {
		rot.value -= 120;
	}
	if (rot.value >= 60) {
		rot.value += 180;
	}

	glm::vec3 min, max;
	min.y = trans.pos.y;
	max.y = trans.pos.y + halfExtents.y * 2.0f;

	const glm::vec2 x = rot * glm::vec2{halfExtents.x * 2.0f, 0};
	const glm::vec2 z = rot * glm::vec2{0, halfExtents.z * 2.0f};

	const glm::vec2 p = {0, 0};
	const glm::vec2 px = x;
	const glm::vec2 pz = z;
	const glm::vec2 pxz = x + z;

	min.x = glm::min(p.x, pz.x);
	min.z = glm::min(p.y, px.y);

	max.x = glm::max(pxz.x, px.x);
	max.z = glm::max(pxz.y, pz.y);

	const glm::vec3 half = {halfExtents.x, 0, halfExtents.z};

	return {min + half, max + half};
}

inline bool FastRayTest2(const glm::vec3 min, const glm::vec3 max,
						 const RayInfo &ray, float &near, glm::vec3 &normal)
{
	assert(glm::all(glm::lessThanEqual(min, max)));
	alignas(16) glm::vec3 bounds[2] = {min, max};
	alignas(16) glm::vec3 tmin, tmax;

	int normalAxis = 0;

	for (int i = 0; i < 2; ++i) {
		tmin[i] = (bounds[ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
		tmax[i] = (bounds[1 - ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
	}

	if ((tmin.x > tmax.y) || (tmin.y > tmax.x))
		return false;

	if (tmin.y > tmin.x) {
		tmin.x = tmin.y;
		normalAxis = 1;
	}

	if (tmax.y < tmax.x) {
		tmax.x = tmax.y;
	}

	for (int i = 2; i < 3; ++i) {
		tmin[i] = (bounds[ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
		tmax[i] = (bounds[1 - ray.signs[i]][i] - ray.start[i]) * ray.invDir[i];
	}

	if ((tmin.x > tmax.z) || (tmin.z > tmax.x)) {
		return false;
	}

	if (tmin.z > tmin.x) {
		normalAxis = 2;
		tmin.x = tmin.z;
	}
	if (tmax.z < tmax.x) {
		tmax.x = tmax.z;
	}
	near = tmin.x;
	float far = tmax.x;

	if (far < 0.0f)
		return false;

	if (near > far)
		return false;

	if (near < 0.0f) {
		normal = -ray.dirNormalized;
		near = 0.0f;
	} else {
		normal = {0, 0, 0};
		normal[normalAxis] = ray.signs[normalAxis] ? 1 : -1;
		assert(glm::dot(normal, ray.dir) < 0);
	}

	return true;
}

bool VertBox::RayTest(const Transform &trans, const RayInfo &ray, float &near,
					  glm::vec3 &normal)
{
	return RayTestFast(trans, trans.ToLocal(ray), near, normal);
}

bool VertBox::RayTestFast(const Transform &trans, const RayInfo &rayInverse,
						  float &near, glm::vec3 &normal)
{
	if (FastRayTest2(-halfExtents, halfExtents, rayInverse, near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool VertBox::CylinderTestOnGround(const Transform &trans, const Cyllinder &cyl,
								   glm::vec3 pos, float &offsetHeight)
{
	pos = trans / pos;

	const glm::vec2 pos2{pos.x, pos.z};
	const glm::vec2 half2{halfExtents.x, halfExtents.z};

	const glm::vec2 d = abs(pos2) - half2;

	if constexpr (false) { // slow
		const float a = glm::length(glm::max(d, glm::vec2{0, 0}));
		const float b = glm::min(glm::maxcomp(d), 0.0f);
		if (a + b - cyl.radius >= 0.0f) {
			return false;
		}
	} else {
		const float b = glm::min(glm::maxcomp(d), 0.0f);
		const float br = cyl.radius - b;
		if (0 >= br) {
			return false;
		}
		const float br2 = br * br;
		const float a2 = glm::length2(glm::max(d, glm::vec2{0, 0}));
		if (a2 >= br2) {
			return false;
		}
	}

	return CylinderTestOnGroundAssumeCollision2D(trans, cyl, pos, offsetHeight);
}

bool VertBox::CylinderTestOnGroundAssumeCollision2D(const Transform &trans,
													const Cyllinder &cyl,
													glm::vec3 pos,
													float &offsetHeight)
{
	offsetHeight = pos.y - trans.pos.y - halfExtents.y * 2.0f;
	return true;
}

bool VertBox::CylinderTestMovement(const Transform &trans,
								   float &validMovementFactor,
								   const Cyllinder &cyl, glm::vec3 from,
								   glm::vec3 to, glm::vec3 &normal)
{
	// TODO
	assert(false);
}

// axis aligned box centered at the origin, with halfSize = "size" and extruded
// by "rad"
float RoundedBoxIntersectXZ(glm::vec2 ro, glm::vec2 rd, glm::vec2 size,
							float rad)
{
	// bounding box
	glm::vec2 m = glm::vec2(1.0) / rd;
	glm::vec2 n = m * ro;
	glm::vec2 k = abs(m) * (size + rad);
	glm::vec2 t1 = -n - k;
	glm::vec2 t2 = -n + k;
	float tN = glm::max(t1.x, t1.y);
	float tF = glm::min(t2.x, t2.y);
	if (tN > tF || tF < 0.0)
		return -1.0;
	float t = tN;

	// convert to first octant
	glm::vec2 pos = ro + t * rd;
	glm::vec2 s = glm::sign(pos);
	ro *= s;  // y -> 0
	rd *= s;  // y -> 0
	pos *= s; // y -> 0

	// faces
	pos -= size;
	if (pos.x < 0.0f && pos.y < 0.0f) {
		return t;
	}

	// some precomputation
	glm::vec2 oc = ro - size; // y -> inf
	glm::vec2 dd = rd * rd;	  // y -> 0
	glm::vec2 oo = oc * oc;	  // y -> inf
	glm::vec2 od = oc * rd;	  // y -> 0 * inf
	float ra2 = rad * rad;	  // y -> 0

	t = 1e20;

	// edge Y
	{
		float a = dd.y + dd.x;
		float b = od.y + od.x;
		float c = oo.y + oo.x - ra2;
		float h = b * b - a * c;
		if (h > 0.0) {
			h = (-b - sqrt(h)) / a;
			if (h > 0.0 && h < t)
				t = h;
		}
	}

	if (t > 1e19)
		return -1;

	return t;
}

// axis aligned box centered at the origin, with dimensions "size" and extruded
// by "rad"
float AabbBoxIntersect_RoundedXZ(glm::vec3 ro, glm::vec3 rd, glm::vec3 size,
								 float _rad, float _height)
{
	const glm::vec3 rad{_rad, 0.0f, _rad};
	size.y += _height / 2.0f;
	ro.y += _height / 2.0f;

	// bounding box
	const glm::vec3 m = glm::vec3(1.0f) / rd;
	const glm::vec3 n = m * ro;
	const glm::vec3 k = glm::abs(m) * (size + rad);
	const glm::vec3 t1 = -n - k;
	const glm::vec3 t2 = -n + k;
	float tN = glm::max(glm::max(t1.x, t1.y), t1.z);
	float tF = glm::min(glm::min(t2.x, t2.y), t2.z);
	if (tN > tF || tF < 0.0f)
		return -1.0f;
	float t = tN;

	// convert to first octant
	glm::vec3 pos = ro + t * rd;
	const glm::vec3 s = glm::sign(pos);
	ro *= s;
	rd *= s;
	pos *= s;

	// faces
	pos -= size;
	pos = glm::max(pos, glm::vec3(pos.y, pos.z, pos.x));
	if (glm::min(glm::min(pos.x, pos.y), pos.z) < 0.0f)
		return t;

	// some precomputation
	const glm::vec3 oc = ro - size;
	const glm::vec3 dd = rd * rd;
	const glm::vec3 oo = oc * oc;
	const glm::vec3 od = oc * rd;
	float ra2 = _rad * _rad;

	t = 1e20f;

	// edge Y
	{
		const float a = dd.z + dd.x;
		const float b = od.z + od.x;
		const float c = oo.z + oo.x - ra2;
		float h = b * b - a * c;
		if (h > 0.0f) {
			h = (-b - sqrt(h)) / a;
			if (t > h && h > 0.0f && glm::abs(ro.y + rd.y * h) < size.y)
				t = h;
		}
	}
	
	if (t > 1e19f)
		t = -1.0f;
	
	if (t < 0.0f) {
		t = 0.0f;
	}

	return t;
}

// normal of a rounded box
glm::vec3 roundedboxNormal(glm::vec3 pos, glm::vec3 siz, float _rad, float _height)
{
	float y = 0.0f;
	const glm::vec3 abspos = glm::abs(pos);
	const glm::vec3 s = glm::sign(pos);
	
	if (abspos.y > siz.y + _height/2.0f) {
		y = pos.y > 0.0f ? 1.0f : -1.0f;
	}
	
	const glm::vec2 absposxz{abspos.x, abspos.z};
	const glm::vec2 sizxz{siz.x, siz.z};
	
	
	glm::vec3 n = s * glm::normalize(glm::max(absposxz - sizxz, glm::vec2(0.0f)));
}

} // namespace Collision3D
