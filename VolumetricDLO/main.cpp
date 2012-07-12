// VolumetricDLO.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TetCell.h"
#include "VolumetricDOO.h"
using namespace Filum;
int _tmain(int argc, _TCHAR* argv[])
{
	/*
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
	*/

	int n = 50;
	vector<vec3<Real> > r;
	for (int idx = 0; idx <n; ++idx)
		r.push_back(vec3<Real>(cos(idx / 10.), sin(idx / 10.), idx / 20));

	VolumetricDOO dlo(r, 0.1, 0.02);

	cin.get();
	return 0;
}

