// The following function is based on GraphicsGems code:
// https://github.com/erich666/GraphicsGems/blob/master/gemsii/RayCPhdron.c
// Based on code by Eric Haines, erich@eye.com

#include "../include/collision3d/CollisionAlgorithms.hpp"

namespace Collision3D
{
bool TestPlaneIterational(glm::vec3 normal, float d, const RayInfo &ray,
						  float &near, float &far, int &frontNormal,
						  int &backNormal, int id)
{
	/* Compute intersection point T and sidedness */
	float vd = glm::dot(ray.dir, normal);
	/* Compute sidedness */
	float vn = glm::dot(ray.start, normal) - d;
	if (vd == 0.0) {
		/* ray is parallel to plane - check if ray origin is inside plane's
		   half-space */
		if (vn > 0.0)
			/* ray origin is outside half-space */
			return false;
	} else {
		/* ray not parallel - get distance to plane */
		float t = -vn / vd;
		if (vd < 0.0) {
			/* front face - T is a near point */
			if (t > far)
				/* hit outside (after) backface */
				return false;
			if (t > near) {
				/* hit near face, update normal */
				frontNormal = id;
				near = t;
			}
		} else {
			/* back face - T is a far point */
			if (t < near)
				/* hit outside (before) near face */
				return false;
			if (t < far) {
				/* hit far face, update normal */
				backNormal = id;
				far = t;
			}
		}
	}
	return true;
}
} // namespace Collision3D
