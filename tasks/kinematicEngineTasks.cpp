/*
 * kinematicEngineTasks.cpp
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#include "services.h"
#include "hardware/robot/robotModel.h"
#include "hardware/robot/robotDescription.h"
#include "kinematicEngineTasks.h"

KinematicEngineTasks::KinematicEngineTasks()
	: m_tasks()
	, m_idleTask(nullptr)
	, m_tree() {

	m_tree.setup(*services.getRobotModel().getRobotDescription());
	std::map<MotorID, Degree> values;
	m_tree.getMotorValues(values);

	for (auto value : values) {
		value.second = 0 * degrees;
	}

	m_tree.setMotorValues(values);
	m_tree.setGyroscopeAngles(arma::eye(3, 3));
}

KinematicEngineTasks::~KinematicEngineTasks() {
}

