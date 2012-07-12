#include "StdAfx.h"
#include "VolumetricDOO.h"

using namespace Filum;

VolumetricDOO::VolumetricDOO(const vector<vec3<Real> > &points, Real radius, Real mass)
{
	nPoints = points.size();
	this->rad = radius;
	this->mass = mass / (3*nPoints);

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
		R[idx].SetMass(this->mass);
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
		Q[idx].SetMass(this->mass);
	}
	// create P
	vec3<Real> p0 = cross(-q0[0], points[1] - points[0]);
	P[0] = MassPoint(points[0] + p0 / length(p0) * cathetus);
	P[0].SetMass(this->mass);
	for (int idx = 1; idx < nPoints; ++idx)
	{
		p0 = cross(q0[idx], points[idx-1] - points[idx]);
		P[idx] = MassPoint(points[idx] + p0 / length(p0) * cathetus);
		P[idx].SetMass(this->mass);
	}
	
	cells = new TetCell[nPoints - 1][3];
	// add tetrahedral cells
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0] = TetCell(&R[idx+1], &P[idx], &Q[idx], &R[idx]);
		cells[idx][1] = TetCell(&R[idx+1], &Q[idx+1], &Q[idx], &P[idx]);
		cells[idx][2] = TetCell(&R[idx+1], &P[idx], &P[idx+1], &Q[idx+1]);

		cout<<cells[idx][0].InitialVolume()<<" "<<cells[idx][1].InitialVolume()<<" "<<cells[idx][2].InitialVolume()<<endl;
	}
}

VolumetricDOO::~VolumetricDOO(void)
{
	delete[] cells;
}

void VolumetricDOO::ResetForces()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].ResetForce();
		Q[idx].ResetForce();
		R[idx].ResetForce();
	}
}

void VolumetricDOO::ResetDisplacements()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].ResetDisplacement();
		Q[idx].ResetDisplacement();
		R[idx].ResetDisplacement();
	}
}

void VolumetricDOO::ResetRestitutionVelocities()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].ResetRestitutionVelocity();
		Q[idx].ResetRestitutionVelocity();
		R[idx].ResetRestitutionVelocity();
	}
}

void VolumetricDOO::ComputeInternalForces()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		cells[idx][0].UpdateForces();
		cells[idx][1].UpdateForces();
		cells[idx][2].UpdateForces();
	}
}

void VolumetricDOO::ComputeCorrectedPositions()
{
	for (int idx = 0 ; idx <nPoints; ++idx)
	{
		P[idx].CorrectPosition();
		Q[idx].CorrectPosition();
		R[idx].CorrectPosition();
	}
}

void VolumetricDOO::ComputeCorrectedVelocites()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].CorrectVelocity();
		Q[idx].CorrectVelocity();
		R[idx].CorrectVelocity();
	}
}

void VolumetricDOO::StepUpdatePositions()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].PositionUpdate();
		Q[idx].PositionUpdate();
		R[idx].PositionUpdate();
	}
}

void VolumetricDOO::StepUpdateVelocities()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].VelocityUpdate();
		Q[idx].VelocityUpdate();
		R[idx].VelocityUpdate();
	}
}

