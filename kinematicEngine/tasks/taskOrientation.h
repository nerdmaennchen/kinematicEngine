/*
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKORIENTATION_H_
#define KINEMATICENGINETASKORIENTATION_H_

#include "task.h"
#include "taskMethodPoint.h"

namespace kinematicEngine {

class TaskOrientation : public Task {
public:

	enum Axis {
		AXIS_X = 0,
		AXIS_Y = 1,
		AXIS_Z = 2
	};

	TaskOrientation();

	TaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree, Axis axis = AXIS_X);

	TaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, const KinematicTree &tree, Axis axis = AXIS_X);

	TaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree, arma::colvec3 target, Axis axis = AXIS_X);

	virtual ~TaskOrientation();

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

	TaskMethodPoint m_methodPoint;
};

}

#endif /* KINEMATICENGINETASKORIENTATION_H_ */
