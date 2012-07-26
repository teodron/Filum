#pragma once
#include "stdafx.h"
namespace Filum
{
	/**
	* \class MassPoint
	* \brief Provides support for a 3D point endowed with a mass
	* 
	* A mass point is described by its position, mass, force and velocity
	* vectors. For easier compatibility with various integration methods,
	* this class also stores the position and velocity of this point
	* at three consecutive time instances: t - Dt, t and t + Dt,
	* where Dt represents the integration time step.
	* Support for collision handling or constraint reinforcement is
	* provided through the aid of a displacements vector dr, while
	* the restitution velocities are stored separately in vRest. 
	* Friction and reaction/restitution forces are accumulated 
	* as usual in the forces vector f.
	*/
	class MassPoint
	{
		friend class TetCell;
		friend ostream& operator<<(ostream& output, const MassPoint& p);

	private:
		/// positions
		vec3<Real> r, rMinus, rPlus, r0;
		/// velocities
		vec3<Real> v, vMinus, vPlus, v0;
		/// collision and constraints contributions
		vec3<Real> vRes, f, dr;

		Real mass; //< the mass concetration at this point (in kilograms)
	public:
		static class TorsionUtilities
		{
			friend class VolumetricDOO;
		private:
			static Real InitialAngleBetweenPoints(MassPoint* Ri, MassPoint* Rj, MassPoint* Qi, MassPoint* Qj)
			{
				return AngleBetweenVectors(Rj->r0 - Ri->r0, Qi->r0 - Ri->r0, Qj->r0 - Rj->r0);
			}
			static Real CurrentAngleBetweenPoints(MassPoint* Ri, MassPoint* Rj, MassPoint* Qi, MassPoint* Qj)
			{
				return AngleBetweenVectors(Rj->rPlus - Ri->rPlus, Qi->rPlus - Ri->rPlus, Qj->rPlus - Rj->rPlus);
			}
			static quat<Real> TorsionQuat(MassPoint* Ri, MassPoint* Rj, MassPoint* Qi, MassPoint* Qj,const Real& initialAngle)
			{
				Real currentAngle = CurrentAngleBetweenPoints(Ri, Rj, Qi, Qj);
				return quat_from_axis_angle(Rj->rPlus - Ri->rPlus, 0.01 *(initialAngle - currentAngle));
			}

			static void ApplyQuat(MassPoint * R, MassPoint * Q, quat<Real> torsionQuat)
			{
				vec3<Real> r = R->rPlus; 
				vec3<Real> q = Q->rPlus;
				q -= r;
				quat<Real> hq = torsionQuat * quat<Real>(q, 0) * conjugate(torsionQuat);
				q = hq.v;
				q += r;
				Q->dr += q - Q->rPlus;
			}
			static void ApplyQuat(MassPoint* R, MassPoint* Q, MassPoint* P, quat<Real> torsionQuat)
			{
				vec3<Real> r = R->rPlus; 
				vec3<Real> q = Q->rPlus;
				vec3<Real> p = P->rPlus;
				q -= r;
				p -= r;
				quat<Real> hq = torsionQuat * quat<Real>(q, 0) * conjugate(torsionQuat);
				quat<Real> hp = torsionQuat * quat<Real>(p, 0) * conjugate(torsionQuat);
				q = hq.v;
				p = hp.v;
				q += r;
				p += r;
				Q->dr += (q - Q->rPlus);
				P->dr += (p - P->rPlus);
			}
			static Real SegLength(MassPoint *Ri, MassPoint *Rj)
			{
				return length(Ri->rPlus - Rj->rPlus);
			}
		};
		/// Creates a static mass point at a specified position
		MassPoint(vec3<Real> pos);
		/// copy constructor
		MassPoint(const MassPoint& src);
		/// empty default constructor
		MassPoint();
		/// mass setter
		void SetMass(Real mass) { this->mass = mass;}

		/// resets the force accumulator
		void ResetForce() { this->f = zeroVec; }

		/// resets the displacement accumulator
		void ResetDisplacement() { this->dr = zeroVec; }

		/// resets the restitution velocity accumulator
		void ResetRestitutionVelocity() {this->vRes = zeroVec; }

		/// Position Verlet Integration Method: one step of the position update equation
		void PositionUpdate() 
		{ 
			rPlus = 2.0 * r - rMinus + f / mass * dTime * dTime;
		}

		/// Position Verlet Integration Method: one step of the velocity update equation
		void VelocityUpdate()
		{
			v = (rPlus - rMinus) * 0.5 / dTime;
		}

		/// Corrects the predicted position by adding the accumulated displacement
		void CorrectPosition()
		{
			rPlus += dr;
		}

		/// Corrects the predicted velocity by adding the accumulated residual/restitution velocity
		void CorrectVelocity()
		{
			vPlus += vRes;
		}

		/// Adds an amount representing an external force contribution
		void AddExternalForce(const vec3<Real>& force)
		{
			this->f += force;
		}

		void AddDampingForce(const Real& b)
		{
			this->f += - b * v;
		}

		/// Synchronizes the positions and velocities at this point by copying the newly computed values into the current holders
		void SynchronizePositionsAndVelocities();

		~MassPoint(void);

		void Perturb(vec3<Real> v)
		{
			this->r += v;
		}
	};

	inline ostream& operator<<(ostream& output, const MassPoint& p) {
		output << "(" <<  p.r.x << ", " << p.r.y << ", "<< p.r.z <<")"<< " -- ";
		output << "(" <<  p.r0.x << ", " << p.r0.y << ", "<< p.r0.z <<")"<<endl;
		return output;  // for multiple << operators.
	}

}

