// VolumetricDLO.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "TetCell.h"
#include "VolumetricDOO.h"
#include "World.h"

using namespace Filum;

void InitWorld(int argc, char* argv[])
{
	cout << "Initializing world .."<<endl;
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
	int n = 50;
	vector<vec3<Real> > r;
	for (int idx = 0; idx <n; ++idx)
		r.push_back(vec3<Real>(3*cos(idx / 3.), 3*sin(idx / 3.), idx / 5.));

	VolumetricDOO dlo(r, 0.2, 2.);
	dlo.SetKl(20);
	dlo.SetKv(10);
	dlo.SetDampingCoefficient(0.9);
	dlo.SetLengthConstraintFraction(0.02);

	World::GetInstance()->SetDLO(&dlo);
	InitWorld(argc, argv);
	
	cin.get();
	
	return 0;
}

