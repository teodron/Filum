#pragma once
#include "stdafx.h"
#include "MassPoint.h"
namespace Filum
{
	/** 
	* \class TetCell
	* \brief Tetrahedral cell containing 4 mass points as its corners
	*
	* The vertices of the tetrahedron are the corners of the cell and 
	* are also part of a more complex object, discretized accordingly.
	* Every cell can query its corners for position, force, velocity
	* values and is also able to update these corners (typically
	* through restitution velocities, displacements and cumulated
	* force contributions). A cell is used as both a collection of
	* linear and volumetric springs and as a collection of distance
	* constraints along its edges.
	*/
	class TetCell
	{
	private:

		/// tetrahedron corners
		MassPoint *pi, *pj, *pk, *pl;

		/// volumetric spring stiffness coefficient
		Real kV;

		/// linear spring stiffness coefficient
		Real kL;

		/** 
			\brief the volumetric force contribution at corner pi 
		*/
		vec3<Real> Fvpi(Real V, Real V0)
		{
			return FVol(.1, V, V0, pi->r, pj->r, pk->r, pl->r);
		}

		/** 
			\brief the volumetric force contribution at corner pj 
		*/
		vec3<Real> Fvpj(Real V, Real V0)
		{
			return FVol(.1, V, V0, pj->r, pi->r, pl->r, pk->r);
		}

		/** 
			\brief the volumetric force contribution at corner pk
		*/
		vec3<Real> Fvpk(Real V, Real V0)
		{
			return FVol(.1, V, V0, pk->r, pi->r, pj->r, pl->r);
		}

		/** 
			\brief the volumetric force contribution at corner pl
		*/
		vec3<Real> Fvpl(Real V, Real V0)
		{
			return FVol(.1, V, V0, pl->r, pi->r, pk->r, pj->r);
		}

	public:
		
		/**
		* \brief Constructs a cell from 4 different mass points
		*
		* To create a consistently oriented cell,
		* start from a corner and list the next 3 corners in a clock-wise
		* order as if their determined triangle is viewed from the initial
		* tetrahedron corner/vertex.
		*/
		TetCell(MassPoint * pi0, MassPoint * pj0, MassPoint * pk0, MassPoint * pl0);
		
		/// setter method for the linear spring constant
		void SetKl(Real value) { kL = value;}

		/// setter method for the volumetric spring constant
		void SetKv(Real value) { kV = value;}

		/// returns the volume of the cell at the current time instance
		Real Volume()
		{
			return ComputeVolume(pi->r, pj->r, pk->r, pl->r);
		}

		/// returns the volume of the cell in its initial rest position
		Real InitialVolume()
		{
			return ComputeVolume(pi->r0, pj->r0, pk->r0, pl->r0);
		}


		void UpdateForces();

	

		void Move(vec3<Real> dr)
		{
			pi->r += dr;
		}
		void UpdatePos()
		{
			pi->r += pi->f;
			pj->r += pj->f;
			pk->r += pk->f;
			pl->r += pl->f;
		}

	};
}