// VolumetricDLO.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TetCell.h"
#include "VolumetricDOO.h"
#include "World.h"
#include <cmath> 

using namespace Filum;

void InitWorld(int argc, char* argv[])
{
	vec3<Real> ri(1,1,0), rj(0,0,0), rk(0,0,1);
	vec3<Real> qi(2,1,0), qj(1,0,0), qk(0,-1,0);

	vec3<Real> nri(0,1,0), nrj(0,0,0), nrk(0,0,1);
	vec3<Real> nqi(0,1,1), nqj(1,0,0), nqk(1,-1,1);

	Real uij = AngleBetweenVectors(rj - ri, qi - ri, qj - rj);
	Real ujk = AngleBetweenVectors(rk - rj, qj - rj, qk - rk);

	Real nuij = AngleBetweenVectors(nrj - nri, nqi - nri, nqj - nrj);
	Real nujk = AngleBetweenVectors(nrk - nrj, nqj - nrj, nqk - nrk);

	vec3<Real> wij = (rj - ri) / length(rj - ri);
	vec3<Real> wjk = (rk - rj) / length(rk - rj);

	vec3<Real> nwij = (nrj - nri) / length(nrj - nri);
	vec3<Real> nwjk = (nrk - nrj) / length(nrk - nrj);
	Real lambda = length(nrj - nri) / (length(nrj - nri) + length(nrk - nrj));

	quat<Real> qij = quat_from_axis_angle(nwij, 0.5*(uij - nuij));

	quat<Real> qjk = conjugate(quat_from_axis_angle(nwjk, 0.5*(ujk - nujk)));

	quat<Real> torsion = slerp(qij, qjk, lambda);
	cout<< " initial "<< uij << " " << ujk << endl;
	cout<< " current "<< nuij << " " << nujk << endl; 
	nqj -= nrj;
	quat<Real>  sj(nqj, 0);
	sj = torsion * sj * conjugate(torsion);
	nqj = sj.v + nrj;

	nuij = AngleBetweenVectors(nrj - nri, nqi - nri, nqj - nrj);
	nujk = AngleBetweenVectors(nrk - nrj, nqj - nrj, nqk - nrk);
	cout<< " corrected "<< nuij << " " << nujk;
	system("pause");
	cout << "Initializing world .."<<endl<< endl;


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(512, 512);
	glutCreateWindow("Filum DLO simulator");
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE); 
	World::GetInstance()->Init();

	glutDisplayFunc(World::Display);
	glutReshapeFunc(World::Reshape);
	glutKeyboardFunc(World::Keyboard);
	glutIdleFunc(World::Display);
	glutMainLoop();
}
int _tmain(int argc, char* argv[])
{
	/**
	MassPoint* pi = new MassPoint(vec3<Real>(0,0,0));
	MassPoint* pj = new MassPoint(vec3<Real>(1,0,0));
	MassPoint* pk = new MassPoint(vec3<Real>(0,1,0));
	MassPoint* pl = new MassPoint(vec3<Real>(0,0,1));
	TetCell tet(pi, pj, pk, pl);
	tet.SetKl(0.3); tet.SetKv(0.1);

	tet.Move(vec3<Real>(1.2,1.2,1.2));
	tet.ResetDr();
	cout<< tet.InitialVolume() << "  " <<tet.Volume() <<endl;
	tet.ComputeLengthConstraintContributions(0.1);
	//tet.UpdateForces();
	tet.UpdatePosDr();
	cout<< tet.InitialVolume() << "  " <<tet.Volume() <<endl;
	/**/
	cin.get();
	

	//return 0;
	int n = 20;
	vector<vec3<Real> > r;
	for (int idx = 0; idx <n; ++idx)
		r.push_back(vec3<Real>(3*cos(idx / 3.), 3*sin(idx / 3.), idx / 5.));

	VolumetricDOO dlo(r, 0.2, 20);
	dlo.SetKl(20);
	dlo.SetKv(20);
	dlo.SetDampingCoefficient(0.9);
	dlo.SetLengthConstraintFraction(0.02);

	World::GetInstance()->SetDLO(&dlo);
	InitWorld(argc, argv);
	
	cin.get();
	
	return 0;
}

