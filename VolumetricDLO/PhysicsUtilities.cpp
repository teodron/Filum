#include "stdafx.h"
#include "PhysicsUtilities.h"
#include <cmath>



inline Real sign(Real a)
{
	if (a == 0) return 0;
	if (a > 0) return 1;
	else return -1;
}

Real Filum::AngleBetweenVectors(const vec3<Real>& w, const vec3<Real>& a, const vec3<Real>& b)
{
	vec3<Real> axis = w / length(w);
	vec3<Real> aPerp = a - dot(a, axis) * axis;
	vec3<Real> bPerp = b - dot(b, axis) * axis;
	Real sigma = sign(dot(cross(aPerp, bPerp), axis));
	Real cosine = (dot(aPerp, bPerp) / (length(aPerp) * length(bPerp)));
	if (cosine > 1)
		cosine = 1;
	if (cosine < -1)
		cosine = -1;
	return sigma * acos(cosine);
}
