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
	// compute initial torsion angles
	torsionAngles.resize(nPoints - 1);
	for (int idx = 0; idx < nPoints -1; ++idx)
	{
		//cout<<idx<< " "<<AngleBetweenVectors(R[idx+1].r - R[idx].r, Q[idx].r - R[idx].r, Q[idx+1].r - R[idx+1].r)<<endl;
		torsionAngles[idx] = MassPoint::TorsionUtilities::InitialAngleBetweenPoints(&R[idx], &R[idx+1], &Q[idx], &Q[idx+1]);
		cout<< idx<<" "<<torsionAngles[idx] <<endl;

	}
	// Reset node/corner dynamic vectors
	ResetForces();
	ResetRestitutionVelocities();
	ResetDisplacements();
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
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0].UpdateForces();
		cells[idx][1].UpdateForces();
		cells[idx][2].UpdateForces();
	}
}

void VolumetricDOO::ComputeExternalForces()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		/**
		P[idx].AddExternalForce(gravity * mass);
		Q[idx].AddExternalForce(gravity * mass);
		R[idx].AddExternalForce(gravity * mass);
		/**/
		// damping
		P[idx].AddDampingForce(bDamping);
		Q[idx].AddDampingForce(bDamping);
		R[idx].AddDampingForce(bDamping);
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

void VolumetricDOO::ComputeLengthConstraints()
{
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0].ComputeLengthConstraintContributions(lengthConstraintFraction);
		cells[idx][1].ComputeLengthConstraintContributions(lengthConstraintFraction);
		cells[idx][2].ComputeLengthConstraintContributions(lengthConstraintFraction);
	}
}

void VolumetricDOO::ComputeTorsionConstraints()
{
	quat<Real> prevQuat;
	quat<Real> currentQuat;
	prevQuat = MassPoint::TorsionUtilities::TorsionQuat(&R[0], &R[1], &Q[0], &Q[1], torsionAngles[0]);
	Real prevLength = MassPoint::TorsionUtilities::SegLength(&R[0], &R[1]);
	Real currLength;
	for (int k = 1; k < nPoints - 1; ++k)
	{
		currentQuat = conjugate(MassPoint::TorsionUtilities::TorsionQuat(&R[k], &R[k+1],&Q[k], &Q[k+1], torsionAngles[k]));
		currLength = MassPoint::TorsionUtilities::SegLength(&R[k], &R[k+1]);
		Real lambda = prevLength / (currLength + prevLength);
		quat<Real> torsionQuat = slerp(prevQuat, currentQuat, lambda);
		MassPoint::TorsionUtilities::ApplyQuat(&R[k], &Q[k], &P[k], torsionQuat);
		prevLength = currLength;
		prevQuat = conjugate(currentQuat);
	}
}

void VolumetricDOO::HandleSelfCollision()
{

}

void VolumetricDOO::HandleExternalCollision()
{

}

void VolumetricDOO::SynchronizePositionsAndVelocities()
{
	for (int idx = 0; idx < nPoints; ++idx)
	{
		P[idx].SynchronizePositionsAndVelocities();
		Q[idx].SynchronizePositionsAndVelocities();
		R[idx].SynchronizePositionsAndVelocities();
	}
}

void VolumetricDOO::Render()
{
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0].Render();
		cells[idx][1].Render();
		cells[idx][2].Render();
	}
}

void VolumetricDOO::PerformUpdateStep()
{
	
	// update force contributions
	ComputeTorsionConstraints();
	ComputeInternalForces();
	ComputeExternalForces();

	// integrate to find new positions
	StepUpdatePositions();

	// compute constraint contributions
	ResetDisplacements();
	
	ComputeLengthConstraints();
	

	// apply displacements
	ComputeCorrectedPositions();

	// find new velocities: central differencing
	StepUpdateVelocities();

	// handle any collisions
	ResetForces();
	ResetRestitutionVelocities();
	ResetDisplacements();
	HandleSelfCollision();
	HandleExternalCollision();

	// correct positions after collision
	ComputeCorrectedPositions();
	// re-estimate velocities
	StepUpdateVelocities();
	// correct the predicted velocities
	ComputeCorrectedVelocites();
	
	// synchronize/update by copying the new amounts into previous time-instances
	SynchronizePositionsAndVelocities();
}

void VolumetricDOO::SetKl(Real value)
{
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0].SetKl(value);
		cells[idx][1].SetKl(value);
		cells[idx][2].SetKl(value);
	}
}

void VolumetricDOO::SetKv(Real value)
{
	for (int idx = 0; idx < nPoints - 1; ++idx)
	{
		cells[idx][0].SetKv(value);
		cells[idx][1].SetKv(value);
		cells[idx][2].SetKv(value);
	}
}

void VolumetricDOO::Perturb()
{
	P[0].Perturb(vec3<Real>(0,0,.01));
}