/*
 * kinematicEngineTaskOrientation.h
 *
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKORIENTATION_H_
#define KINEMATICENGINETASKORIENTATION_H_

#include "kinematicEngineTask.h"
#include "kinematicEngineTaskMethodPoint.h"

class KinematicEngineTaskOrientation : public KinematicEngineTask {
public:

	enum Axis {
		AXIS_X = 0,
		AXIS_Y = 1,
		AXIS_Z = 2
	};

	KinematicEngineTaskOrientation();

	KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree, Axis axis = AXIS_X);

	KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, const KinematicTree &tree, Axis axis = AXIS_X);

	KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree, arma::colvec3 target, Axis axis = AXIS_X);

	virtual ~KinematicEngineTaskOrientation();

	/**
	 * get the jacobian for this task
	 *
	 * @param kinematicTree
	 * @param normalizeJacobian
	 *
	 * @return
	 */
	virtual arma::mat getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian = false) const;

	virtual arma::colvec getError(const KinematicTree &kinematicTree) const;

	virtual void setTarget(arma::colvec3 target);

private:
	Axis m_axis;

	MotorID m_referenceCoordinateSystem;

	KinematicEngineTaskMethodPoint m_methodPoint;
};

#endif /* KINEMATICENGINETASKORIENTATION_H_ */
