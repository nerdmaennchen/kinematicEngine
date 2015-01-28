/*
 * kinematicEngineTaskCOMWholeRobot.h
 *
 *  Created on: 28.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKCOMWHOLEROBOT_H_
#define KINEMATICENGINETASKCOMWHOLEROBOT_H_

#include "kinematicEngineTask.h"

class KinematicEngineTaskCOMWholeRobot : public KinematicEngineTask {
public:

	typedef int SubDimension;

	KinematicEngineTaskCOMWholeRobot();

	KinematicEngineTaskCOMWholeRobot(std::string name, MotorID base, KinematicTree const &tree);

	KinematicEngineTaskCOMWholeRobot(std::string name, MotorID base, MotorID reference, KinematicTree const &tree);

	virtual ~KinematicEngineTaskCOMWholeRobot();

	virtual arma::colvec getError(const KinematicTree &kinematicTree) const;

	/**
	 * get the jacobian for this task
	 *
	 * @param kinematicTree
	 * @param normalizeJacobian
	 *
	 * @return
	 */
	virtual arma::mat getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normJacobian = false) const;

private:
	MotorID m_referenceCoordinateSystem;

	arma::mat getJacobianForTaskSub(const KinematicTree &kinematicTree,
			const KinematicNode *node,
			bool traversingUp,
			KinematicMass &o_massFromSubTree) const;
};

#endif /* KINEMATICENGINETASKCOMWHOLEROBOT_H_ */
