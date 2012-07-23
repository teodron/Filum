#include "stdafx.h"
#include "PhysicsUtilities.h"
#include <cmath>



inline Real sign(Real a)
{
	if (a == 0) return 0;
	if (a > 0) return 1;
	else return -1;
}

Real Filum::AngleBetweenVectors(vec3<Real> w, vec3<Real> a, vec3<Real> b)
{
	w /= length(w);
	vec3<Real> aPerp = a - dot(a, w) * w;
	vec3<Real> bPerp = b - dot(b, w) * w;
	Real sigma = sign(dot(cross(aPerp, bPerp), w));
	Real cosine = (dot(aPerp, bPerp) / (length(aPerp) * length(bPerp)));
	if (cosine > 1)
		cosine = 1;
	if (cosine < -1)
		cosine = -1;
	return sigma * acos(cosine);
}
