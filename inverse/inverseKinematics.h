/*
 * inverserKinematicsh.h
 *
 *  Created on: 25.06.2014
 *      Author: lutz
 */

#ifndef INVERSERKINEMATICSH_H_
#define INVERSERKINEMATICSH_H_

#include <representations/motion/kinematicTree.h>
#include <representations/motion/kinematicengine/kinematicEngineTasks.h>
#include <representations/motion/kinematicengine/kinematicEngineConstraints.h>

#include "tools/kinematicEngine/tasks/kinematicEngineTask.h"
#include "tools/kinematicEngine/tasks/kinematicEngineTaskDefaultPosition.h"

class InverseKinematics {
public:
	InverseKinematics();
	virtual ~InverseKinematics();


	void setMaxValueChange(double newVal) {
		m_maxValueChange = newVal;
	}

	virtual double iterationStep(
			KinematicTree const& tree
			, std::map<MotorID, Degree>& o_angles
			, KinematicEngineTasksContainer const& tasks
			, KinematicEngineConstraints const& constraints
			, const KinematicEngineTaskDefaultPosition* idleTask = nullptr
			, bool printDebugInfos = false
		) const = 0;

	virtual double calculateSpeeds(
			KinematicTree const& tree
			, std::map<MotorID, RPM>& o_speeds
			, KinematicEngineTasksContainer const& tasks
			, KinematicEngineConstraints const& constraints
			, const KinematicEngineTaskDefaultPosition* idleTask = nullptr
		) const = 0;

	virtual double iterationStepGravitation(
			KinematicTree const& tree
			, std::map<MotorID, double>& torques
			, std::vector<const KinematicEngineTask*> const&
		) const = 0;

protected:
	/**
	 * how much change can be generated for each joint per iteration this value represents eg. angle or revolution
	 */
	double m_maxValueChange;
};

#endif /* INVERSERKINEMATICSH_H_ */
