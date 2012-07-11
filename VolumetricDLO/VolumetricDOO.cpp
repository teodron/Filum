#include "StdAfx.h"
#include "VolumetricDOO.h"

using namespace Filum;

VolumetricDOO::VolumetricDOO(const vector<vec3<Real> > &points, Real radius)
{
	nPoints = points.size();
	this->rad = radius;
	Real cathetus = sqrt(2.0) * 0.5 * radius;

	// reserve mass point vectors
	P.resize(nPoints);
	Q.resize(nPoints);
	R.resize(nPoints);
	vector<vec3<Real> > q0;
	q0.resize(nPoints);

	for (int idx = 0; idx < nPoints; ++idx)
	{
		R[idx] = MassPoint(points[idx]);
		//cout<<R[idx];
	}
	
	vec3<Real> R0R1 = points[1] - points[0];
	R0R1 /= length(R0R1);
	vec3<Real> qi(R0R1.z, R0R1.x, R0R1.y);
	qi = cross(R0R1, qi);
	for (int idx = 1; idx < nPoints - 1; ++idx)
	{
		vec3<Real> cp = cross(points[idx - 1] - points[idx], points[idx+1] - points[idx]);
		if (length(cp) > epsilon)
		{
			qi = cp;
			break;
		}
	}
	qi /= length(qi);
	q0[0] = qi;
	for (int idx = 1; idx < nPoints -1; ++idx)
	{
		vec3<Real> cp = cross(points[idx-1] - points[idx], points[idx+1] - points[idx]);
		if (length(cp) > epsilon)
		{
			q0[idx] = cp / length(cp);
		}
		else
		{
			q0[idx] = q0[idx-1];
		}
	}
	q0[nPoints-1] = q0[nPoints-2];
	// create Q
	for (int idx = 0; idx < nPoints; ++idx)
	{
		Q[idx] = MassPoint(points[idx] + cathetus * q0[idx] / length(q0[idx]));
	}
	// create P
	vec3<Real> p0 = cross(-q0[0], points[1] - points[0]);
	P[0] = MassPoint(points[0] + p0 / length(p0) * cathetus);
	for (int idx = 1; idx < nPoints; ++idx)
	{
		p0 = cross(q0[idx], points[idx-1] - points[idx]);
		P[idx] = MassPoint(points[idx] + p0 / length(p0) * cathetus);
	}
	
}


VolumetricDOO::~VolumetricDOO(void)
{
}
