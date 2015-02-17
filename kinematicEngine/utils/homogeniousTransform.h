/*
 * homogeniousTransform.h
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#ifndef SRC_KINEMATICENGINE_UTIL_HOMOGENIOUSTRANSFORM_H_
#define SRC_KINEMATICENGINE_UTIL_HOMOGENIOUSTRANSFORM_H_

#include "utils/units.h"
#include <armadillo>

namespace kinematicEngine {

namespace homogeneousTransform
{
	arma::mat44 getTransform(Meter dx, Meter dy, Meter dz, Degree alphaX, Degree alphaY, Degree alphaZ);

	arma::mat44 invertHomogeneous(arma::mat44 const& _mat);
}

}


#endif /* SRC_KINEMATICENGINE_UTIL_HOMOGENIOUSTRANSFORM_H_ */
