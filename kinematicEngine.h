/*
 * kinematicEngine.h
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_H_
#define KINEMATICENGINE_H_

#include "kinematicEngine/kinematicTree.h"
#include "kinematicEngine/tasks/kinematicEngineTasks.h"
#include "kinematicEngine/constraints/kinematicEngineConstraints.h"

#include "kinematicEngine/inverse/inverseKinematicJacobian.h"

#include <random>

class KinematicEngine  {
public:
	KinematicEngine();
	virtual ~KinematicEngine();

	void setRobotModel(const RobotDescription* robotDescription);

	std::map<MotorID, double> solveIKStepSpeeds(std::map<MotorID, double> curValues,
												std::map<MotorID, double> curSpeeds,
												KinematicEngineTasks& tasks,
												KinematicEngineConstraints const& constraints);

	std::map<MotorID, double> solveIKStepValues(std::map<MotorID, double> curValues,
												std::map<MotorID, double> curSpeeds,
												KinematicEngineTasks& tasks,
												KinematicEngineConstraints const& constraints,
												int iterationCnt, double& error);

	void setEpsilon(double epsilon);
	void setSpeedEpsilon(double epsilon);
	void setNullspaceEpsilon(double epsilon);
	void setMaxValueChange(double maxValueChange);
	void setValueNoise(double valueNoise);

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

	void fillTreeWithValues(KinematicTree &tree,
							std::map<MotorID, double> values,
							std::map<MotorID, double> speeds,
							bool addNoise = false);
};

#endif /* KINEMATICENGINE_H_ */
