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
		vec3<Real> r, rMinus, rPlus, r0;
		vec3<Real> v, vMinus, vPlus, v0;
		vec3<Real> vRes, f, dr;
		Real mass; //< the mass concetration at this point (in kilograms)
	public:
		/// Creates a static mass point at a specified position
		MassPoint(vec3<Real> pos);
		~MassPoint(void);
	};

	inline ostream& operator<<(ostream& output, const MassPoint& p) {
		output << "(" <<  p.r.x << ", " << p.r.y << ", "<< p.r.z <<")"<< " -- ";
		output << "(" <<  p.r0.x << ", " << p.r0.y << ", "<< p.r0.z <<")"<<endl;
		return output;  // for multiple << operators.
	}

}

