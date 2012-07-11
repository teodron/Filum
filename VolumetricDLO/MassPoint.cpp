#include "StdAfx.h"
#include "MassPoint.h"

using namespace Filum;


MassPoint::MassPoint()
{

}
MassPoint::MassPoint(vec3<Real> pos)
{
	// initialize
	r = r0 = rPlus = rMinus = pos;
	v = v0 = vPlus = vMinus = zeroVec;
	dr = f = vRes = zeroVec;
}

MassPoint::MassPoint(const MassPoint& src)
{
	// copy 
	r = src.r; r0 = src.r0; rMinus = src.rMinus; rPlus = src.rPlus;
	v = src.v; v0 = src.v0; vMinus = src.vMinus; vPlus = src.vPlus;
	dr = src.dr; f = src.f; vRes = src.vRes;
}
MassPoint::~MassPoint(void)
{
}
