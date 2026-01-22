// This file is part of Collision3D.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../SpatialPartitioning/include/spatial_partitioning/BvhMedianSplitHeap.hpp"

#include "../include/collision3d/CollisionShapes_AnyOrCompound.hpp"

namespace Collision3D
{
using _ =
	spp::BvhMedianSplitHeap<spp::Aabb, uint32_t, uint32_t, 0, 1, void>;

CompoundPrimitive::~CompoundPrimitive()
{
	if (bvh) {
		delete bvh;
		bvh = nullptr;
	}
}

void CompoundPrimitive::Optimise()
{
	if (bvh) {
		delete bvh;
		bvh = nullptr;
	}
	if (primitives.size < 12) {
		return;
	}
	bvh = new BvhType(primitives.size + 1);
	bvh->StartFastAdding();
	for (int i = 0; i < primitives.size; ++i) {
		const auto &s = primitives[i];
		Aabb aabb = s.GetAabb({});
		bvh->Add(i + 1, aabb, 1);
	}
	bvh->StopFastAdding();
	bvh->Rebuild();
}

spp::Aabb CompoundPrimitive::GetAabb(const Transform &trans) const
{
	if (bvh) {
		return bvh->GetTotalAabb();
	} else {
		spp::Aabb aabb = spp::AABB_INVALID;
		for (const auto &s : primitives) {
			aabb = aabb + s.GetAabb(trans);
		}
		return aabb;
	}
}

bool CompoundPrimitive::RayTest(const Transform &trans, const RayInfo &ray,
								float &near, glm::vec3 &normal) const
{
	if (RayTestLocal(trans.ToLocal(ray), near, normal)) {
		normal = trans.rot * normal;
		return true;
	} else {
		return false;
	}
}

bool CompoundPrimitive::RayTestLocal(const RayInfo &ray, float &near,
									 glm::vec3 &normal) const
{
	if (bvh) {
		assert(false && "Untested");
		struct CallBack : public spp::RayCallback<spp::Aabb, uint32_t, uint32_t, 0> {
			glm::vec3 &normal;
			const CompoundPrimitive *cp;
			bool hasHit = false;
		} cb{{}, normal, this};
		cb.mask = ~(uint32_t)0;
		cb.broadphase = (BvhType*)bvh;
			
		typedef spp::RayPartialResult (*CbT)(spp::RayCallback<spp::Aabb, uint32_t, uint32_t, 0> *,
											 uint32_t);
		cb.callback =
			(CbT) +
			[](CallBack *cb, uint32_t entity) -> spp::RayPartialResult {
			const auto &prim = cb->cp->primitives[entity-1];
			float ne;
			glm::vec3 no;
			if (prim.RayTestLocal(*cb, ne, no)) {
				// TODO: remove reduntant code
				if (ne < 0.0f) {
					ne = 0.0f;
				}
				if (ne <= cb->cutFactor) {
					cb->normal = no;
					cb->cutFactor = ne;
					cb->hasHit = true;
					return {ne, true};
				}
			}
			return {1.0f, false};
		};
		
		((RayInfo&)cb) = ray;
		bvh->IntersectRay_const(cb);
		near = cb.cutFactor;
		return cb.hasHit;
	} else {
		bool res = false;
		float ne;
		glm::vec3 no;
		for (const auto &s : primitives) {
			if (s.RayTestLocal(ray, ne, no)) {
				if (res) {
					if (near > ne) {
						near = ne;
						normal = no;
					}
				} else {
					near = ne;
					normal = no;
					res = true;
				}
			}
		}
		return res;
	}
}

bool CompoundPrimitive::CylinderTestOnGround(const Transform &trans,
											 const Cylinder &cyl, glm::vec3 pos,
											 float &offsetHeight,
											 glm::vec3 *onGroundNormal,
											 bool *isOnEdge) const
{
	// TODO: implement with bvh
	bool res = false;
	float ofh;
	for (const auto &s : primitives) {
		if (s.CylinderTestOnGround(trans, cyl, pos, ofh, onGroundNormal,
								   isOnEdge)) {
			if (res) {
				if (offsetHeight < ofh) {
					offsetHeight = ofh;
				}
			} else {
				offsetHeight = ofh;
				res = true;
			}
		}
	}
	return res;
}

bool CompoundPrimitive::CylinderTestMovement(const Transform &trans,
											 float &validMovementFactor,
											 const Cylinder &cyl,
											 const RayInfo &movementRay,
											 glm::vec3 &normal) const
{
	if (bvh) {
		assert(false && "Untested");
		struct _Cb : public spp::AabbCallback<spp::Aabb, uint32_t, uint32_t, 0> {
			const CompoundPrimitive *cp;
			
			const Transform &trans;
			float &validMovementFactor;
			const Cylinder &cyl;
			const RayInfo &movementRay;
			glm::vec3 &normal;
			bool res = false;
		} cb{{}, this,
			trans,
			validMovementFactor,
			cyl,
			movementRay, normal};
		cb.mask = ~(uint32_t)0;

		typedef void (*CbT)(spp::AabbCallback<spp::Aabb, uint32_t, uint32_t, 0> *, uint32_t);
		cb.callback = (CbT) + [](_Cb *cb, uint32_t entity) {
			const auto &prim = cb->cp->primitives[entity-1];
			
			float vmf;
			glm::vec3 no;
			if (prim.CylinderTestMovement(cb->trans, vmf, cb->cyl, cb->movementRay, no)) {
				// TODO: remove reduntant code
				if (cb->res) {
					if (cb->validMovementFactor > vmf) {
						cb->validMovementFactor = vmf;
						cb->normal = no;
					}
				} else {
					cb->validMovementFactor = vmf;
					cb->normal = no;
					cb->res = true;
				}
			}
		};
		
		Aabb aabb = cyl.GetAabb({movementRay.start - trans.pos, {}});
		aabb.max += glm::max({0, 0, 0}, movementRay.dir) + ON_EDGE_FACTOR;
		aabb.min += glm::min({0, 0, 0}, movementRay.dir) - ON_EDGE_FACTOR;
		cb.aabb = aabb;
		
		bvh->IntersectAabb(cb);

		return cb.res;
	} else {
		bool res = false;
		float vmf;
		glm::vec3 no;
		for (const auto &s : primitives) {
			if (s.CylinderTestMovement(trans, vmf, cyl, movementRay, no)) {
				if (res) {
					if (validMovementFactor > vmf) {
						validMovementFactor = vmf;
						normal = no;
					}
				} else {
					validMovementFactor = vmf;
					normal = no;
					res = true;
				}
			}
		}
		return res;
	}
}
} // namespace Collision3D
