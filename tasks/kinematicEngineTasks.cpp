/*
 * kinematicEngineTasks.cpp
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#include "kinematicEngineTasks.h"

KinematicEngineTasks::KinematicEngineTasks(RobotDescription const& description)
	: m_tasks()
	, m_idleTask(nullptr)
	, m_tree() {
	m_tree.setup(description);
}

KinematicEngineTasks::~KinematicEngineTasks() {
}

