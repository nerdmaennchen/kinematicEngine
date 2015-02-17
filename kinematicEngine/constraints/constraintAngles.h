/*
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_CONSTRAINTS_CONSTRAINTANGLES_H_
#define KINEMATICENGINE_CONSTRAINTS_CONSTRAINTANGLES_H_

#include "constraint.h"

namespace kinematicEngine {

class ConstraintAngles : public Constraint {
public:
	ConstraintAngles();
	ConstraintAngles(KinematicTree const& tree);
	virtual ~ConstraintAngles();

	virtual arma::colvec getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const override;

private:
	arma::colvec m_minValues;
	arma::colvec m_maxValues;
	int m_numMotors;
};

}

#endif
