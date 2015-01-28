/*
 * inverseKinematicJacobian.h
 *
 *  Created on: 13.06.2014
 *      Author: lutz
 */

#ifndef INVERSEKINEMATICJACOBIAN_H_
#define INVERSEKINEMATICJACOBIAN_H_

#include "inverseKinematics.h"

#include <vector>
#include <random>

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
			, std::map<MotorID, Degree>& o_angles
			, KinematicEngineTasksContainer const& tasks
			, KinematicEngineConstraints const& constraints
			, const KinematicEngineTaskDefaultPosition* idleTask = nullptr
			, bool printDebugInfos = false
		) const;

	virtual double calculateSpeeds(
			KinematicTree const& tree
			, std::map<MotorID, RPM>& o_speeds
			, KinematicEngineTasksContainer const& tasks
			, KinematicEngineConstraints const& constraints
			, const KinematicEngineTaskDefaultPosition* idleTask = nullptr
		) const;

	virtual double iterationStepGravitation(
			KinematicTree const& tree
			, std::map<MotorID, double>& torques
			, std::vector<const KinematicEngineTask*> const&
		) const;


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


	arma::mat getJacobianForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks, arma::mat &jacobianRAW, bool normalize = false) const;
	arma::colvec getErrorForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks) const;
	arma::colvec getSpeedTargetForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks) const;

	int calculateNumRows(std::vector<const KinematicEngineTask*> const& tasks) const;

	arma::mat buildPseudoInverse(arma::mat matrix, double epsilon) const;
};

#endif /* INVERSEKINEMATICJACOBIAN_H_ */
