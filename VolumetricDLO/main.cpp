// VolumetricDLO.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TetCell.h"
using namespace Filum;
int _tmain(int argc, _TCHAR* argv[])
{
	MassPoint* pi = new MassPoint(vec3<Real>(0,0,0));
	MassPoint* pj = new MassPoint(vec3<Real>(1,0,0));
	MassPoint* pk = new MassPoint(vec3<Real>(0,1,0));
	MassPoint* pl = new MassPoint(vec3<Real>(0,0,1));
	TetCell tet(pi, pj, pk, pl);
	tet.Move(vec3<Real>(1.5, 1.50, 1.50));
	vec3<Real> force = tet.Fvpi(tet.Volume(), tet.InitialVolume());
	tet.Move(force);
	cout<< tet.Volume()<< " " << tet.InitialVolume()<<" "<< force.x<<" "<<force.y<< " "<<force.z<<endl;
	cin.get();
	return 0;
}

