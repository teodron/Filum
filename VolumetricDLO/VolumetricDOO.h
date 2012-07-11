#pragma once
#include "stdafx.h"
#include "MassPoint.h"
#include "TetCell.h"
namespace Filum
{
	/**
	* \class VolumetricDOO
	* 
	* \brief Volumetric linear deformable object 
	*
	* This provides support for a deformable one-dimensional object composed of
	* volumetric cells. Tetrahedral cells are endowed with volumetric and linear springs
	* and are essential to providing torsion resistance to a linear object.
	*/
	class VolumetricDOO
	{
	private:
		/// central mass nodes - coincide with a discretized DLO curve image
		vector<MassPoint> R;
		/// normal-like vector 
		vector<MassPoint> P;
		/// binormal-like vector 
		vector<MassPoint> Q;

		/// number of node points of this discrete model
		int nPoints;
		/// the radius of this model ( the circumscribed circle radius for each right RQP triangle)
		Real rad;

		/// tetrahedral cells - 3 for each R_i R_{i+1} segment
		TetCell (*cells)[3];
	public:

		/// construct DLO from a sample set of points and a buffer radius
		VolumetricDOO(const vector<vec3<Real> > & points, Real radius);

		/// clean-up own resources
		~VolumetricDOO(void);
	};
}