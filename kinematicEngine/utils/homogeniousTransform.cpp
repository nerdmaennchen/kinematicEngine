/*
 * homogeniousTransform.cpp
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */


#include "homogeniousTransform.h"

namespace kinematicEngine {

namespace homogeneousTransform
{
	arma::mat44 getTransform(Meter dx, Meter dy, Meter dz, Degree alphaX, Degree alphaY, Degree alphaZ) {
		arma::mat44 translation = arma::eye(4, 4);
		translation(0, 3) = dx.value();
		translation(1, 3) = dy.value();
		translation(2, 3) = dz.value();

		/* align z axis to current joint this might incorporate two rotations (first the rotation around x-axis is performed, then around the y axis and around the z axis) */
		arma::mat44 rotX = arma::eye(4, 4);
		arma::mat44 rotY = arma::eye(4, 4);
		arma::mat44 rotZ = arma::eye(4, 4);

		if (std::abs(alphaX.value()) >= std::numeric_limits<double>::epsilon())
		{
			const double cRotX = cos(alphaX);
			const double sRotX = sin(alphaX);
			rotX(1, 1) = cRotX;
			rotX(1, 2) = -sRotX;
			rotX(2, 1) = sRotX;
			rotX(2, 2) = cRotX;
		}

		if (std::abs(alphaY.value()) >= std::numeric_limits<double>::epsilon())
		{
			const double cRotY = cos(alphaY);
			const double sRotY = sin(alphaY);
			rotY(0, 0) = cRotY;
			rotY(0, 2) = sRotY;
			rotY(2, 0) = -sRotY;
			rotY(2, 2) = cRotY;
		}

		if (std::abs(alphaZ.value()) >= std::numeric_limits<double>::epsilon())
		{
			const double cRotZ = cos(alphaZ);
			const double sRotZ = sin(alphaZ);
			rotZ(0, 0) = cRotZ;
			rotZ(0, 1) = -sRotZ;
			rotZ(1, 0) = sRotZ;
			rotZ(1, 1) = cRotZ;
		}

		return translation * rotX * rotY * rotZ;
	}


	arma::mat44 invertHomogeneous(arma::mat44 const& _mat) {
		arma::mat44 ret = arma::eye(4, 4);
		ret.submat(0, 0, 2, 2) = _mat.submat(0, 0, 2, 2).t();
		ret.col(3).rows(0, 2) = -ret.submat(0, 0, 2, 2) * _mat.col(3).rows(0, 2);
		return ret;
	}
}

}
