/*
 * kinematicEngine.cpp
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#include "kinematicEngine.h"

#define DEFAULT_EPSILON (0.001)
#define DEFAULT_SPEED_EPSILON (0.001)
#define DEFAULT_NULLSPACE_EPSILON (0.000000001)
#define DEFAULT_MAX_VALUE_CHANGE (M_PI / 4)
#define DEFAULT_VALUE_NOISE (0.001)

namespace kinematicEngine {

KinematicEngine::KinematicEngine()
	: m_kinematicTree()
	, m_inverseKinematic()
	, generator()
	, m_epsilon(DEFAULT_EPSILON)
	, m_speedEpsilon(DEFAULT_SPEED_EPSILON)
	, m_nullspaceEpsilon(DEFAULT_NULLSPACE_EPSILON)
	, m_maxValueChange(DEFAULT_MAX_VALUE_CHANGE)
	, m_valueNoise(DEFAULT_VALUE_NOISE)
{
	m_inverseKinematic.setEpsilon(m_epsilon);
	m_inverseKinematic.setSpeedEpsilon(m_speedEpsilon);
	m_inverseKinematic.setNullspaceEpsilon(m_nullspaceEpsilon);
}

KinematicEngine::~KinematicEngine() {
}


void KinematicEngine::setEpsilon(double epsilon)
{
	m_epsilon = epsilon;
	m_inverseKinematic.setEpsilon(m_epsilon);
}
void KinematicEngine::setSpeedEpsilon(double epsilon)
{
	m_speedEpsilon = epsilon;
	m_inverseKinematic.setSpeedEpsilon(m_speedEpsilon);
}

void KinematicEngine::setNullspaceEpsilon(double epsilon)
{
	m_nullspaceEpsilon = epsilon;
	m_inverseKinematic.setNullspaceEpsilon(m_nullspaceEpsilon);
}

void KinematicEngine::setMaxValueChange(double maxValueChange)
{
	m_maxValueChange = maxValueChange;
	m_inverseKinematic.setMaxValueChange(m_maxValueChange);
}

void KinematicEngine::setValueNoise(double valueNoise)
{
	m_valueNoise = valueNoise;
}

void KinematicEngine::setOvershootValues(kinematics::MotorValuesMap overshootValues)
{
	m_overshootValues = overshootValues;
}

void KinematicEngine::setRobotModel(const RobotDescription* robotDescription)
{
	m_kinematicTree.setup(*(robotDescription));
}

void KinematicEngine::fillTreeWithValues(KinematicTree &tree,
										kinematics::MotorValuesMap values,
										kinematics::MotorValuesMap speeds,
										bool addNoise)
{
    std::uniform_real_distribution<double> distribution(-m_valueNoise, m_valueNoise);

    for (std::pair<const MotorID, double>& value : values) {
		double noise = 0.;
		if (addNoise) {
			noise = distribution(generator);
		}
		value.second += noise;
    }

    for (std::pair<const kinematics::NodeID, KinematicNode*> nodeP : tree.getNodes()) {
		KinematicNode * node = nodeP.second;
		kinematics::MotorIDs const& motors = node->getMotors();

		kinematics::MotorValuesMap valuesForNode;
		kinematics::MotorValuesMap valueDerivativesForNode;

		for (MotorID const& id : motors) {
			if (values.find(id) != values.end()) {
				// this motor is accessible for inverse kinematics
				valuesForNode[id] = values.at(id);
			}
			if (speeds.find(id) != speeds.end()) {
				valueDerivativesForNode[id] = speeds.at(id);
			}
		}

		node->setValues(valuesForNode);
		node->setValueDerivatives(valueDerivativesForNode);
    }
}

std::map<MotorID, double> KinematicEngine::solveIKStepSpeeds(kinematics::MotorValuesMap curValues,
												kinematics::MotorValuesMap curSpeeds,
												Tasks& tasks,
												Constraints const& constraints)
{
	fillTreeWithValues(tasks.getTree(), curValues, curSpeeds, false);
	std::map<MotorID, double> newSpeeds;
	m_inverseKinematic.calculateSpeeds(tasks.getTree(), newSpeeds, tasks.getTasks(), constraints, tasks.getIdleTask());
	return newSpeeds;
}


std::map<MotorID, double> KinematicEngine::solveIKStepValues(kinematics::MotorValuesMap curValues,
												kinematics::MotorValuesMap curSpeeds,
												Tasks& tasks,
												Constraints const& constraints,
												int iterationCnt, double& error)
{
	std::map<MotorID, double> newValues = curValues;
	for (int i(0); i < iterationCnt; ++i) {
		fillTreeWithValues(tasks.getTree(), newValues, curSpeeds, (i == 0));
		error = m_inverseKinematic.iterationStep(tasks.getTree(), newValues, tasks.getTasks(), constraints, tasks.getIdleTask());
	}
	return newValues;
}

std::map<MotorID, double> KinematicEngine::solveIKGravityOvershoot(kinematics::MotorValuesMap curValues,
												kinematics::MotorValuesMap curSpeeds,
												Tasks& tasks,
												kinematicEngine::Task& task,
												double& error)
{
	fillTreeWithValues(tasks.getTree(), curValues, curSpeeds, false);
	std::map<MotorID, double> torques;

	error = m_inverseKinematic.iterationStepGravitation(tasks.getTree(), torques, task);

	for (std::pair<kinematics::NodeID, KinematicNode*> const& node : tasks.getTree().getNodes())
	{
		kinematics::MotorIDs const& motorIDs = node.second->getMotors();
		for (MotorID motor : motorIDs)
		{
			torques[motor] = utils::limited(torques[motor], -m_overshootValues[motor], m_overshootValues[motor]);
		}
	}

	return torques;
}

}
