/*
 * kinematicEngine.h
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_KINEMATICENGINE_H_
#define KINEMATICENGINE_KINEMATICENGINE_H_

#include "kinematicEngine/kinematicTree.h"
#include "kinematicEngine/tasks/tasks.h"
#include "kinematicEngine/constraints/constraint.h"

#include "kinematicEngine/inverse/inverseKinematicJacobian.h"

#include <random>

namespace kinematicEngine {

class KinematicEngine  {
public:
	KinematicEngine();
	virtual ~KinematicEngine();

	void setRobotModel(const RobotDescription* robotDescription);

	std::map<MotorID, double> solveIKStepSpeeds(std::map<MotorID, double> curValues,
												std::map<MotorID, double> curSpeeds,
												Tasks& tasks,
												Constraints const& constraints);

	std::map<MotorID, double> solveIKStepValues(std::map<MotorID, double> curValues,
												std::map<MotorID, double> curSpeeds,
												Tasks& tasks,
												Constraints const& constraints,
												int iterationCnt, double& error);

	std::map<MotorID, double> solveIKGravityOvershoot(kinematics::MotorValuesMap curValues,
												kinematics::MotorValuesMap curSpeeds,
												Tasks& tasks,
												kinematicEngine::Task& task,
												double& error);
	void setEpsilon(double epsilon);
	void setSpeedEpsilon(double epsilon);
	void setNullspaceEpsilon(double epsilon);
	void setMaxValueChange(double maxValueChange);
	void setValueNoise(double valueNoise);
	void setOvershootValues(kinematics::MotorValuesMap overshootValues);

	KinematicTree const& getKinematicTree() const
	{
		return m_kinematicTree;
	}

private:
	KinematicTree m_kinematicTree;

	InverseKinematicJacobian m_inverseKinematic;

	std::default_random_engine generator;

	/**
	 * epsilon used in the regularisation term to build the pseudo inverse of the jacobian
	 * when calculating the target values
	 */

	double m_epsilon;

	/**
	 * epsilon used in the regularisation term to build the pseudo inverse of the jacobian
	 * when calculating the target speeds
	 */
	double m_speedEpsilon;

	/**
	 * epsilon used in the regularisation term to build the pseudo inverse of the jacobian
	 * when calculating the nullspace of a task set
	 */
	double m_nullspaceEpsilon;

	/**
	 * maximum value change for one iteration
	 */
	double m_maxValueChange;

	/**
	 * parameter for noise generator.
	 * at each solveIKStep call some noise is added to the current values to "hop" out of singular configurations
	 */
	double m_valueNoise;

	/**
	 * paramter for overshoot generation
	 * motor values are overshot when they act against gravity
	 */
	kinematics::MotorValuesMap m_overshootValues;

	void fillTreeWithValues(KinematicTree &tree,
							kinematics::MotorValuesMap values,
							kinematics::MotorValuesMap speeds,
							bool addNoise = false);
};

}

#endif
