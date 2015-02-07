/*
 * kinematicEngineConstraint.h
 *
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINT_H_
#define TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINT_H_

#include "../kinematicTree.h"

class KinematicEngineConstraint {
public:
	/*
	 * if target angles are the desired configuration of the robot (given as tree) then calculate the "error" of angles which cannot be set
	 * this usually gives the clipping error of the target angles given some constraint
	 */
	virtual arma::colvec getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const = 0;

	virtual ~KinematicEngineConstraint() {}
};

#endif /* TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINT_H_ */
