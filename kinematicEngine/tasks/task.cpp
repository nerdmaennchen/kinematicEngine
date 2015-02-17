/*
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#include "task.h"

namespace kinematicEngine {

Task::Task():
		m_name("uninitialized"),
		m_baseNode(MOTOR_NONE),
		m_effectorNode(MOTOR_NONE),
		m_method(nullptr),
		m_speed(0.),
		m_actuatorsToIgnore(),
		m_hasSpeed(false),
		m_weight(0),
		m_precision(0.),
		m_invPath(),
		m_path()
{}

Task::Task(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree) :
	m_name(name),
	m_baseNode(baseNode),
	m_effectorNode(effectorNode),
	m_method(nullptr),
	m_speed(0.),
	m_actuatorsToIgnore(),
	m_hasSpeed(false),
	m_weight(1.),
	m_precision(0.),
	m_invPath(tree.getPathFromNodeToNode(baseNode, effectorNode)),
	m_path(m_invPath)
{
	m_invPath = invertKinematicPath(m_path);
}

Task::Task(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree, TaskMethod *method) :
	m_name(name),
	m_baseNode(baseNode),
	m_effectorNode(effectorNode),
	m_method(method),
	m_speed(0.),
	m_actuatorsToIgnore(),
	m_hasSpeed(false),
	m_weight(1.),
	m_precision(0.),
	m_invPath(tree.getPathFromNodeToNode(baseNode, effectorNode)),
	m_path(m_invPath)
{
	m_invPath = invertKinematicPath(m_path);
}

Task::~Task() {
}

}
