#pragma once
#include "stdafx.h"
namespace Filum
{
	class MassPoint
	{
		friend class TetCell;
		friend ostream& operator<<(ostream& output, const MassPoint& p);
	private:
		vec3<Real> r, rMinus, rPlus, r0;
		vec3<Real> v, vMinus, vPlus, v0;
		vec3<Real> vRes, f, dr;
	public:
		MassPoint(vec3<Real> pos);
		~MassPoint(void);
	};

	inline ostream& operator<<(ostream& output, const MassPoint& p) {
		output << "(" <<  p.r.x << ", " << p.r.y << ", "<< p.r.z <<")"<< " -- ";
		output << "(" <<  p.r0.x << ", " << p.r0.y << ", "<< p.r0.z <<")"<<endl;
		return output;  // for multiple << operators.
	}

}

