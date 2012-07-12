#pragma once
#include "stdafx.h"
#include "MassPoint.h"
#include "TetCell.h"
namespace Filum
{
	/**
	* \class VolumetricDOO
	* 
	* \brief Volumetric linear deformable object 
	*
	* This provides support for a deformable one-dimensional object composed of
	* volumetric cells. Tetrahedral cells are endowed with volumetric and linear springs
	* and are essential to providing torsion resistance to a linear object.
	*/
	class VolumetricDOO
	{
	private:
		/// central mass nodes - coincide with a discretized DLO curve image
		vector<MassPoint> R;
		/// normal-like vector 
		vector<MassPoint> P;
		/// binormal-like vector 
		vector<MassPoint> Q;

		/// number of node points of this discrete model
		int nPoints;
		/// the radius of this model ( the circumscribed circle radius for each right RQP triangle)
		Real rad;
		/// the total DLO mass
		Real mass;
		/// a fraction describing how much a length preserving constraint uses of the residual displacement
		Real lengthConstraintFraction;
		/// tetrahedral cells - 3 for each R_i R_{i+1} segment
		TetCell (*cells)[3];

		/// Computes internal forces at each mass-point by updating the tetrahedral cells that point is part of
		void ComputeInternalForces();

		/// Applies position corrections according to accumulated penalty displacement vectors
		void ComputeCorrectedPositions();

		/// Applies velocity corrections using restitution residual velocities
		void ComputeCorrectedVelocites();

		/// Resets all mass point force vector accumulators
		void ResetForces();

		/// Resets all mass point displacement accumulators
		void ResetDisplacements();

		/// Resets all mass point restitution velocity accumulators
		void ResetRestitutionVelocities();

		/// Updates positions via an integration step 
		void StepUpdatePositions();

		/// Updates velocities via an integration step
		void StepUpdateVelocities();

	public:

		/// construct DLO from a sample set of points and a buffer radius
		VolumetricDOO(const vector<vec3<Real> > & points, Real radius, Real mass);

		/// clean-up own resources
		~VolumetricDOO(void);
	};
}