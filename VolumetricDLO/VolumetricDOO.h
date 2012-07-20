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
		/// linear spring stiffness constant
		Real Kl;
		/// volumetric spring stiffness constant
		Real Kv;
		/// damping coefficient
		Real bDamping;

		/// tetrahedral cells - 3 for each R_i R_{i+1} segment
		TetCell (*cells)[3];

		/// Computes internal forces at each mass-point by updating the tetrahedral cells that point is part of
		void ComputeInternalForces();

		/// Computes the force contribution from external force fields (e.g. gravity)
		void ComputeExternalForces();

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

		/// Computes the length constraints induced penalty displacements for each cell corner
		void ComputeLengthConstraints();

		/// Computes the local torsion constraints contributions updating the Q - P nodes adjacent to each R node
		void ComputeTorsionConstraints();

		/**
		* \brief Performs self collision detection and resolution
		* In case of a collision pair, restitution/reaction and friction
		* forces are accumulated, restitution velocities are computed from impulses
		* and the displacement accumulators are update in order to best
		* separate the colliding cell edges.
		*/
		void HandleSelfCollision();

		/**
		* \brief Performs collision resolution with other, external objects
		* External objects are typically composed of cylinders.
		* \todo Implement this function using an external object pointer
		*/
		void HandleExternalCollision();

		/**
		* \brief Pushes the values from newer position/velocity holders into their past holders
		* The final/corrected positions and velocities at instance t are copied into the t - Dt instance
		* and those of instance t + Dt are copied into the t instance value holders. This process
		* is performed for all nodes of the DLO
		*/
		void SynchronizePositionsAndVelocities();

	public:

		/// construct DLO from a sample set of points and a buffer radius
		VolumetricDOO(const vector<vec3<Real> > & points, Real radius, Real mass);

		/// clean-up own resources
		~VolumetricDOO(void);

		void SetKl(Real value);
		void SetKv(Real value);
		void SetLengthConstraintFraction(Real value)
		{
			lengthConstraintFraction = value;
		}
		void SetDampingCoefficient(Real value)
		{
			bDamping = value;
		}

		/// renders the DLO using deprecated but simple triangle calls
		void Render();

		/**
		* \brief Single step update procedure
		*
		* Updates the cell corners by computing force contributions and then 
		* calculating new positions and velocities from a Verlet integration step
		* and from collision and internal constraint responses.
		*/
		void PerformUpdateStep();

		void Perturb();
	};
}