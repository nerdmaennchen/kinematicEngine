/*
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKLOCATION_H_
#define KINEMATICENGINETASKLOCATION_H_

#include "task.h"

namespace kinematicEngine {

class TaskLocation : public Task {
	typedef kinematics::NodeID NodeID;
public:
	TaskLocation();

	TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree);

	TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, NodeID referenceNode, const KinematicTree &tree);

	TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree, TaskMethod *method);

	virtual ~TaskLocation();

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

private:
	MotorID m_referenceCoordinateSystem;

};

}
#endif /* KINEMATICENGINETASKLOCATION_H_ */
