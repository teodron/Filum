#include "stdafx.h"
namespace Filum
{
	inline Real ComputeVolume( const vec3<Real>& pi, const vec3<Real>& pj,
						              const vec3<Real>& pk, const vec3<Real>& pl)
	{
		return 1.0 / 6.0 * dot(pj - pi, cross(pk - pi, pl - pi));
	}
	inline vec3<Real> FVol(Real kV, Real  V,Real V0, const vec3<Real>& pi, const vec3<Real>& pj,
						              const vec3<Real>& pk, const vec3<Real>& pl)
	{
		return kV / (6.0*V0*V0) * (V - V0) * cross(pj - pl, pk - pl);
	}
}