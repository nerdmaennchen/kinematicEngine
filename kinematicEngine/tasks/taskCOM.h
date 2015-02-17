/*
 *  Created on: 25.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICTASKCOM_H_
#define KINEMATICTASKCOM_H_

#include "task.h"

namespace kinematicEngine {

class TaskCOM : public Task {
public:
	TaskCOM();

	TaskCOM(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree);
	virtual ~TaskCOM();


	/**
	 * get the jacobian for this task
	 * @param kinematicTree
	 * @param normalizeJacobian
	 * @return
	 */
	virtual arma::mat getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian = false) const;

	virtual arma::colvec getError(const KinematicTree &kinematicTree) const;

private:
	MotorID m_referenceNode;
};

}
#endif /* KINEMATICTASKCOM_H_ */
