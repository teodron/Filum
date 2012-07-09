#include "StdAfx.h"
#include "MassPoint.h"

using namespace Filum;

MassPoint::MassPoint(vec3<Real> pos)
{
	// initialize
	r = r0 = rPlus = rMinus = pos;
	v = v0 = vPlus = vMinus = zeroVec;
	dr = f = vRes = zeroVec;
}


MassPoint::~MassPoint(void)
{
}
