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
			return FVol(.1, V, V0, pi->r, pj->r, pk->r, pl->r);
		}
		vec3<Real> Fvpj(Real V, Real V0)
		{
			return FVol(.1, V, V0, pj->r, pi->r, pl->r, pk->r);
		}
		vec3<Real> Fvpk(Real V, Real V0)
		{
			return FVol(.1, V, V0, pk->r, pi->r, pj->r, pl->r);
		}
		vec3<Real> Fvpl(Real V, Real V0)
		{
			return FVol(.1, V, V0, pl->r, pi->r, pk->r, pj->r);
		}
		void Movepi(vec3<Real> dr)
		{
			pi->r += dr;
		}
		void Move(vec3<Real> dr)
		{
			pl->r += dr;
		}
		TetCell(MassPoint * pi0, MassPoint * pj0, MassPoint * pk0, MassPoint * pl0);

	};
}