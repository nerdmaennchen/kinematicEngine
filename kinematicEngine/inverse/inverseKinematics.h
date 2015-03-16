/*
 * inverserKinematicsh.h
 *
 *  Created on: 25.06.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_INVERSERKINEMATICSH_H_
#define KINEMATICENGINE_INVERSERKINEMATICSH_H_

#include "kinematicEngine/kinematicTree.h"
#include "kinematicEngine/tasks/tasks.h"
#include "kinematicEngine/constraints/constraint.h"

#include "kinematicEngine/tasks/task.h"
#include "kinematicEngine/tasks/taskDefaultPosition.h"

#include "kinematicEngine/kinematics.h"

namespace kinematicEngine {

class InverseKinematics {
protected:
	typedef kinematics::NodeID NodeID;
	typedef kinematics::MotorValuesMap MotorValuesMap;
public:
	InverseKinematics();
	virtual ~InverseKinematics();


	void setMaxValueChange(double newVal) {
		m_maxValueChange = newVal;
	}

	virtual double iterationStep(
			KinematicTree const& tree
			, MotorValuesMap& o_values
			, TasksContainer const& tasks
			, Constraints const& constraints
			, const TaskDefaultPosition* idleTask = nullptr
		) const = 0;

	virtual double calculateSpeeds(
			KinematicTree const& tree
			, MotorValuesMap& o_values
			, TasksContainer const& tasks
			, Constraints const& constraints
			, const TaskDefaultPosition* idleTask = nullptr
		) const = 0;

	virtual double iterationStepGravitation(
			KinematicTree const& tree
			, std::map<MotorID, double>& torques
			, Task const& task
		) const = 0;

protected:
	/**
	 * how much change can be generated for each joint per iteration this value represents eg. angle or revolution
	 */
	double m_maxValueChange;
};

}

#endif
