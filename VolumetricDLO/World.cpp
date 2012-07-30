#include "stdafx.h"
#include "World.h"
#include "ShaderController.h"

using namespace Filum;


World* World::instance = NULL;

vec3<Real> camPos(100,0,50);
vec3<Real> lookAt(0,0,0);
vec3<Real> upVector(0,0,1);

shared_ptr<FXZShader> vertexShader, fragmentShader;
shared_ptr<FXZProgram> mainShaderProgram;

World::World(void)
{
}
World* World::GetInstance()
{
	if (instance == NULL)
		instance = new World();
	return instance;
}

void World::Display()
{
	glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable (GL_DEPTH_TEST);

	glLoadIdentity();
	gluLookAt(camPos.x, camPos.y, camPos.z, lookAt.x, lookAt.y, lookAt.z, upVector.x, upVector.y, upVector.z);

	glColor3f(1,0,0);

	World::GetInstance()->dlo->Render();
	World::GetInstance()->dlo->PerformUpdateStep();

	glutSwapBuffers();
}

void World::Reshape(int w, int h)
{
	glViewport (0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective (60, (GLfloat)w / (GLfloat)h, .1, 1000.0);
	glMatrixMode (GL_MODELVIEW);
}

void World::Keyboard(unsigned char key, int x, int y)
{
	if (key == 27)
		exit(1);
	if (key == 'q')
	{
		camPos += vec3<Real>(0,0,1);
		lookAt += vec3<Real>(0,0,1);
	}
	if (key == 'e')
	{
		camPos += vec3<Real>(0,0,-1);
		lookAt += vec3<Real>(0,0,-1);
	}
	if (key == 'w')
	{
		camPos += vec3<Real>(-1,0,0);
		lookAt += vec3<Real>(-1,0,0);
	}
	if (key == 's')
	{
		camPos += vec3<Real>(1,0,0);
		lookAt += vec3<Real>(1,0,0);
	}
	if (key == 'a')
	{
		camPos += vec3<Real>(0,-1,0);
		lookAt += vec3<Real>(0,-1,0);
	}
	if (key == 'd')
	{
		camPos += vec3<Real>(0,1,0);
		lookAt += vec3<Real>(0,1,0);
	}
	// update dlo
	if (key == 'u')
	{
		World::GetInstance()->dlo->PerformUpdateStep();
	}
	// perturb dlo
	if (key == 'p')
	{
		World::GetInstance()->dlo->Perturb();
	}
}

void World::Init()
{
		// initialize shader system
	glewInit();

if (glewIsSupported("GL_VERSION_2_0"))
		printf("Ready for OpenGL 2.0\n");
	else {
		printf("OpenGL 2.0 not supported\n");
		exit(1);
	}
	 std::cout << glutGet(GLUT_WINDOW_NUM_SAMPLES);  // here it says '0'

	

	mainShaderProgram.reset(new FXZProgram());
	shared_ptr<FXZShader> vertexShader(new FXZShader(GL_VERTEX_SHADER));
	vertexShader->setSource("vshader.vert");
	vertexShader->compileShader();
	vertexShader->infoLog();
	shared_ptr<FXZShader> fragmentShader(new FXZShader(GL_FRAGMENT_SHADER));
	fragmentShader->setSource("fshader.frag");
	fragmentShader->compileShader();
	fragmentShader->infoLog();

	mainShaderProgram->attachShader(vertexShader.get());
	mainShaderProgram->attachShader(fragmentShader.get());
	mainShaderProgram->linkProgram();
	mainShaderProgram->infoLog();
	mainShaderProgram->useProgram();

	mainShaderProgram->addUniform("lightDir", UNIFORM3);
	mainShaderProgram->setUniform("lightDir", 1, 0 , 0);
}

World::~World(void)
{
}
