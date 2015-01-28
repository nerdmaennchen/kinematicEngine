/*
 * odeUtils.h
 *
 *  Created on: 26.05.2014
 *      Author: lutz
 */

#ifndef ODEUTILS_H_
#define ODEUTILS_H_

#include <armadillo>
#include <ode/ode.h>

class odeUtils {

public:

	static void getRotationMatrixAsDMat(const arma::mat44 &mat, dReal *o_mat) {
		for (uint row = 0; row < 3; ++row) {
			dReal* rowPtr = &(o_mat[row * 4]);
			for (uint col = 0; col < 3; ++col) {
				rowPtr[col] = mat(row, col);
			}
			rowPtr[3] = 0.;
		}
	}

	static void getPositionAsDVec(const arma::mat44 &mat, dReal *o_vec) {
		for (uint row = 0; row < 3; ++row) {
			o_vec[row] = mat(row, 3);
		}
		o_vec[3] = 0.;
	}

	static arma::mat44 getMatFromDMatAndDVec(const dReal *i_rotMat, const dReal *i_posVec) {
		arma::mat44 retMat = arma::eye(4, 4);
		for (uint row = 0; row < 3; ++row) {
			const dReal* rowPtr = &(i_rotMat[row * 4]);
			for (uint col = 0; col < 3; ++col) {
				retMat(row, col) = rowPtr[col];
			}
		}
		for (uint row = 0; row < 3; ++row) {
			retMat(row, 3) = i_posVec[row];
		}
		return retMat;
	}
};

#endif /* ODEUTILS_H_ */
