/*
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_CONSTRAINTS_CONSTRAINT_H_
#define KINEMATICENGINE_CONSTRAINTS_CONSTRAINT_H_

#include "kinematicEngine/kinematicTree.h"

namespace kinematicEngine {

class Constraint {
public:
	/*
	 * if target angles are the desired configuration of the robot (given as tree) then calculate the "error" of angles which cannot be set
	 * this usually gives the clipping error of the target angles given some constraint
	 */
	virtual arma::colvec getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const = 0;

	virtual ~Constraint() {}
};

typedef std::vector<Constraint*> Constraints;

}

#endif
