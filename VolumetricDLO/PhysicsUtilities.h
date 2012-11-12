#pragma once
#include "stdafx.h"
namespace Filum
{
	/**
	* \brief Signum template function
	* \param val - the value for which the sign is computed
	* \returns 1 0 or -1, according to the sign of the value
	*/
	template <typename T> int sgn(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	/** 
	* \brief Computes the volume of a tetrahedron
	* The volume is an orientation dependent quantity and it is positive
	* if the clock-wise listing of the face opposite to pi reads
	* (pj, pk, pl). Hence, to create a consistently oriented cell,
	* start from a corner and list the next 3 corners in a clock-wise
	* order as if their determined triangle is viewed from the initial
	* tetrahedron corner/vertex.
	* \param pi is the position of the first tet corner
	* \param pj is the second vertex of the tet cell according to the described convention
	* \param pk is the third corner/vertex
	* \param pl is the fourth/last tet vertex
	* \return the signed tet volume (negative if the (pj, pk, pl) listing is CCW
	*/
	inline Real ComputeVolume( const vec3<Real>& pi, const vec3<Real>& pj,
						              const vec3<Real>& pk, const vec3<Real>& pl)
	{
		return 1.0 / 6.0 * dot(pj - pi, cross(pk - pi, pl - pi));
	}
	/**
	* \brief Volumetric spring force
	* 
	* Although the volume of a cell can be negative due to an orientation convention,
	* the force tends to re-establish the absolute value of this volume. Thus,
	* only the absolute value of the input volume V is used in the computation
	* when compared to the initial V0 volume. If the input volume is indeed negative
	* the resulting force needs to point in the opposite direction because of the 
	* orientation inversion (it points along the normal of the (pj, pk, pl) face,
	* provided the orientation is volume consistent; otherwise, the direction is
	* reversed).
	* \param kV is the volumetric spring stiffness
	* \param V is the current cell volume (in a deformed state)
	* \param V0 is the initial cell volume (undeformed/rest state)
	* \param pi - first cell corner
	* \param pj - second cell corner, following the clock-wise convention
	* \param pk - third cell corner
	* \param pl - fourth cell corner
	*/
	inline vec3<Real> FVol(Real kV, Real  V,Real V0, const vec3<Real>& pi, const vec3<Real>& pj,
						              const vec3<Real>& pk, const vec3<Real>& pl)
	{
		// volumetric force
		// return  kV / (6.0*V0*V0) * (V - V0) * cross(pj - pl, pk - pl);
		// nonlinear spring
		return ( kV / (6.0*V0*V0) * (V - V0) - kV * ( sgn(V0) * (V0 * V0) / fabs(V) - V)) 
			   * cross(pj - pl, pk - pl);
	}

	/**
	* \brief Linear spring force between two pi pj nodes/points
	* \param kL - linear spring stiffness constant
	* \param D - current spring length (deformed state)
	* \param D0 - initial spring legnth (undeformed state)
	* \return elastic spring force
	*/
	inline vec3<Real> FLin(Real kL, Real D, Real D0, const vec3<Real>& pi, const vec3<Real>& pj)
	{
		return - kL / (D0*D0) * (D - D0) * (pi - pj) / D;
	}

	/**
	* \brief Computes the angle between two vectors relative to a common rotation axis
	* The relative angle is the minimum rotation angle against an axis such that the first
	* vector is aligned with the second.
	* \param w - the common rotation axis
	* \param a - the first vector
	* \param b - the second vector
	* \return the signed relative angle of the two vectors (in radians)
	*/
	Real AngleBetweenVectors(const vec3<Real>& w, const vec3<Real>& a, const vec3<Real>& b);
}