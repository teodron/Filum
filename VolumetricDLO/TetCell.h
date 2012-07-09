#pragma once
#include "stdafx.h"
#include "MassPoint.h"
namespace Filum
{
	class TetCell
	{
	private:

		MassPoint *pi, *pj, *pk, *pl;

	public:
		Real Volume()
		{
			 return ComputeVolume(pi->r, pj->r, pk->r, pl->r);
		}
		Real InitialVolume()
		{
			return ComputeVolume(pi->r0, pj->r0, pk->r0, pl->r0);
		}
		vec3<Real> Fvpi(Real V, Real V0)
		{
			return FVol(5, V, V0, pi->r, pj->r, pk->r, pl->r);
		}
		void Move(vec3<Real> dr)
		{
			pi->r += dr;
			cout<<pi->r.x << " "<<pi->r.y << " "<<pi->r.z<<endl;
		}
		TetCell(MassPoint * pi0, MassPoint * pj0, MassPoint * pk0, MassPoint * pl0);

	};
}