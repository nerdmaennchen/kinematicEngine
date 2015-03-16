/*
 *  Created on: 28.09.2014
 *      Author: lutz
 */

#include "taskDefaultPosition.h"

namespace kinematicEngine {

TaskDefaultPosition::TaskDefaultPosition()
	: m_weight(1.)
	, m_speed(0.)
{
}

TaskDefaultPosition::~TaskDefaultPosition() {
}


void TaskDefaultPosition::setDefaultValues(KinematicTree const& tree, kinematics::MotorValuesMap defaultValues) {
	uint motorCnt = tree.getMotorCnt();
	m_defaultValues = arma::zeros(motorCnt);

	std::map<kinematics::NodeID, KinematicNode*> const& nodes = tree.getNodes();
	for (std::pair<kinematics::NodeID, KinematicNode*> const node : nodes) {
		if (node.second->isServo()) {
			kinematics::Motor2IntMap const& m2iMap = node.second->getMotor2IntMap();
			node.second->clipValues(defaultValues);
			for (kinematics::Motor2Int const& m2i : m2iMap) {
				m_defaultValues(m2i.second) = defaultValues[m2i.first];
			}
		}
	}
}


arma::colvec TaskDefaultPosition::getErrors(KinematicTree const& tree) const
{
	uint motorCnt = tree.getMotorCnt();
	arma::colvec curValues = arma::zeros(motorCnt);

	std::map<kinematics::NodeID, KinematicNode*> const& nodes = tree.getNodes();
	for (std::pair<kinematics::NodeID, KinematicNode*> const node : nodes) {
		if (node.second->isServo()) {
			kinematics::Motor2IntMap const& m2iMap = node.second->getMotor2IntMap();
			kinematics::MotorValuesMap const& nodesValues = node.second->getValues();

			for (kinematics::Motor2Int const& m2i : m2iMap) {
				curValues(m2i.second) = nodesValues.at(m2i.first);
			}
		}
	}

	return (m_defaultValues - curValues) * m_weight;
}

}

