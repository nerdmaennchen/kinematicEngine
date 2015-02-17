/*
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#include "tasks.h"

namespace kinematicEngine {

Tasks::Tasks(RobotDescription const& description)
	: m_tasks()
	, m_idleTask(nullptr)
	, m_tree() {
	m_tree.setup(description);
}

Tasks::~Tasks() {
}

}
