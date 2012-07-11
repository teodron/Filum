#pragma once
#include "stdafx.h"
#include "MassPoint.h"
#include "TetCell.h"
namespace Filum
{
	class VolumetricDOO
	{
	private:
		vector<MassPoint> R;
		vector<MassPoint> P;
		vector<MassPoint> Q;

		int nPoints;

		Real rad;
	public:
		VolumetricDOO(const vector<vec3<Real> > & points, Real radius);
		~VolumetricDOO(void);
	};
}