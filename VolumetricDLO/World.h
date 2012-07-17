#pragma once

#include "VolumetricDOO.h"
namespace Filum
{
	
	class World
	{
		World(void);
		static World * instance;
		VolumetricDOO * dlo;
	public:
		VolumetricDOO * GetDLO() { return dlo; }
		void SetDLO(VolumetricDOO * dlo) { this->dlo = dlo;} 

		void Init();

		static World* GetInstance();

	    static void Display();
		static void Reshape(int w, int h);
		static void Keyboard(unsigned char key, int x, int y);
		~World(void);

	};
}
