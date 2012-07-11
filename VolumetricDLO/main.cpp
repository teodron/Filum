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
	tet.SetKl(0.3); tet.SetKv(0.1);

	tet.Move(vec3<Real>(1.2,1.2,1.2));
	cout<< tet.InitialVolume() << "  " <<tet.Volume() <<endl;
	tet.UpdateForces();
	tet.UpdatePos();
	cout<< tet.InitialVolume() << "  " <<tet.Volume() <<endl;
	cin.get();
	return 0;
}

