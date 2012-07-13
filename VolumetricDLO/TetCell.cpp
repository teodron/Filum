
#include "stdafx.h"
#include "TetCell.h"
using namespace Filum;

TetCell::TetCell(MassPoint * pi0, MassPoint * pj0, MassPoint * pk0, MassPoint * pl0)
{
	pi = (pi0);
	pj = (pj0);
	pk = (pk0);
	pl = (pl0);
	if (this->InitialVolume() < 0)
	{
		cout << "negative volume " << endl;
		MassPoint* swapPtr = pi;
		pi = pj;
		pj = swapPtr;
	}
}

TetCell::TetCell(const TetCell& src)
{
	pi = src.pi;
	pj = src.pj;
	pk = src.pk;
	pl = src.pl;
	kL = src.kL;
	kV = src.kV;
}

void TetCell::UpdateForces()
{

	// linear forces
	Real D = 0, D0 = 0;
	vec3<Real> force;
	/**/
	// pi - pj
	D = length(pi->r - pj->r); D0 = length(pi->r0 - pj->r0);
	force = FLin(kL, D, D0, pi->r, pj->r);
	pi->f += force; 
	pj->f += -force;
	// pi - pk
	D = length(pi->r -pk->r); D0 = length(pi->r0 - pk->r0);
	force = FLin(kL, D, D0, pi->r, pk->r);
	pi->f += force;
	pk->f += -force;
	// pi - pj
	D = length(pi->r - pl->r); D0 = length(pi->r0 - pl->r0);
	force = FLin(kL, D, D0, pi->r, pl->r);
	pi->f += force;
	pl->f += -force;
	// pj - pk
	D = length(pj->r - pk->r); D0 = length(pj->r0 - pk->r0);
	force = FLin(kL, D, D0, pj->r, pk->r);
	pj->f += force;
	pk->f += -force;
	// pj - pl
	D = length(pj->r - pl->r); D0 = length(pj->r0 - pl->r0);
	force = FLin(kL, D, D0, pj->r, pl->r);
	pj->f += force;
	pl->f += -force;
	// pk - pl
	D = length(pk->r - pl->r); D0 = length(pk->r0 - pl->r0);
	force = FLin(kL, D, D0, pk->r, pl->r);
	pk->f += force;
	pl->f += -force;
	/**/
	/**
	// volumetric forces
	Real V = 0, V0 = 0;
	V = this->Volume();
	V0 = this->InitialVolume();
	// pi
	pi->f += Fvpi(V, V0);
	// pj
	pj->f += Fvpj(V, V0);
	// pk
	pk->f += Fvpk(V, V0);
	// pl
	pl->f += Fvpl(V, V0);
	/**/
	
}

void TetCell::ComputeLengthConstraintContributions(const Real & fraction)
{
	Real currLength = 0, restLength = 0;
	vec3<Real> displacement(0,0,0);
	// pi - pj 
	currLength = length(pi->rPlus - pj->rPlus);
	restLength = length(pi->r0 - pj->r0);
	displacement = fraction * (pi->rPlus - pj->rPlus) / currLength * (currLength - restLength);
	pi->dr += displacement;
	pj->dr += -displacement;
	// pi - pk
	currLength = length(pi->rPlus - pk->rPlus);
	restLength = length(pi->r0 - pk->r0);
	displacement = fraction * (pi->rPlus - pk->rPlus) / currLength * (currLength - restLength);
	pi->dr += displacement;
	pk->dr += -displacement;
	// pi - pl
	currLength = length(pi->rPlus - pl->rPlus);
	restLength = length(pi->r0 - pl->r0);
	displacement = fraction * (pi->rPlus - pl->rPlus) / currLength * (currLength - restLength);
	pi->dr += displacement;
	pl->dr += -displacement;
	// pj - pk
	currLength = length(pj->rPlus - pk->rPlus);
	restLength = length(pj->r0 - pk->r0);
	displacement = fraction * (pj->rPlus - pk->rPlus) / currLength * (currLength - restLength);
	pj->dr += displacement;
	pk->dr += -displacement;
	// pj - pl
	currLength = length(pj->rPlus - pl->rPlus);
	restLength = length(pj->r0 - pl->r0);
	displacement = fraction * (pj->rPlus - pl->rPlus) / currLength * (currLength - restLength);
	pj->dr += displacement;
	pl->dr += -displacement;
	// pk - pl
	currLength = length(pk->rPlus - pl->rPlus);
	restLength = length(pk->r0 - pl->r0);
	displacement = fraction * (pk->rPlus - pl->rPlus) / currLength * (currLength - restLength);
	pk->dr += displacement;
	pl->dr += -displacement;
}