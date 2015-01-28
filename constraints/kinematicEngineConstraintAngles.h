/*
 * kinematicEngineConstraintAngles.h
 *
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINTANGLES_H_
#define TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINTANGLES_H_

#include "kinematicEngineConstraint.h"

class KinematicEngineConstraintAngles : public KinematicEngineConstraint {
public:
	KinematicEngineConstraintAngles();
	KinematicEngineConstraintAngles(KinematicTree const& tree);
	virtual ~KinematicEngineConstraintAngles();

	virtual arma::colvec getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const override;

private:
	arma::colvec m_minValues;
	arma::colvec m_maxValues;
	int m_numMotors;
};

#endif /* TOOLS_KINEMATICENGINE_CONSTRAINTS_KINEMATICENGINECONSTRAINTANGLES_H_ */
