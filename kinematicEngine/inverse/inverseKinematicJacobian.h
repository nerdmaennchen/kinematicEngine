/*
 * inverseKinematicJacobian.h
 *
 *  Created on: 13.06.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_INVERSEKINEMATICJACOBIAN_H_
#define KINEMATICENGINE_INVERSEKINEMATICJACOBIAN_H_

#include "inverseKinematics.h"

#include <vector>
#include <random>

namespace kinematicEngine {

/*
 * inverse kinematics with dampened least squares method
 * the dampening factor is m_epsilon and should be chosen carefully
 */
class InverseKinematicJacobian : public InverseKinematics {
public:
	InverseKinematicJacobian();
	virtual ~InverseKinematicJacobian();

	virtual double iterationStep(
			KinematicTree const& tree
			, MotorValuesMap& o_values
			, TasksContainer const& tasks
			, Constraints const& constraints
			, const TaskDefaultPosition* idleTask = nullptr
		) const override;

	double calculateSpeeds(
			KinematicTree const& tree
			, MotorValuesMap& o_values
			, TasksContainer const& tasks
			, Constraints const& constraints
			, const TaskDefaultPosition* idleTask = nullptr
		) const override;

	void setEpsilon(double newEpsilon) {
		m_epsilon = newEpsilon;
	}

	void setSpeedEpsilon(double newEpsilon) {
		m_speedEpsilon = newEpsilon;
	}

	void setNullspaceEpsilon(double newEpsilon) {
		m_nullspaceEpsilon = newEpsilon;
	}

protected:
	/**
	 * the epsilon used in the regularisation term when generating the pseudo inverse jacobian
	 */
	double m_epsilon;

	double m_speedEpsilon;

	double m_nullspaceEpsilon;


	arma::mat getJacobianForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks, arma::mat &jacobianRAW, bool normalize = false) const;
	arma::colvec getErrorForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks) const;
	arma::colvec getSpeedTargetForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks) const;

	int calculateNumRows(std::vector<const Task*> const& tasks) const;

	arma::mat buildPseudoInverse(arma::mat matrix, double epsilon) const;
};

}

#endif
