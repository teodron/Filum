#pragma once
#include "stdafx.h"
namespace Filum
{
	class MassPoint
	{
		friend class TetCell;
	private:
		vec3<Real> r, rMinus, rPlus, r0;
		vec3<Real> v, vMinus, vPlus, v0;
		vec3<Real> vRes, f, dr;
	public:
		MassPoint(vec3<Real> pos);
		~MassPoint(void);
	};
}