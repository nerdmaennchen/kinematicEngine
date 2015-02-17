/*
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#include "constraintAngles.h"

namespace kinematicEngine {

ConstraintAngles::ConstraintAngles()
	: m_minValues(arma::zeros(1))
	, m_maxValues(arma::zeros(1))
	, m_numMotors(0)
{}

ConstraintAngles::ConstraintAngles(KinematicTree const& tree)
{
	m_numMotors = tree.getMotorCnt();
	m_minValues = arma::zeros(m_numMotors);
	m_maxValues = arma::zeros(m_numMotors);

	for (std::pair<kinematics::NodeID, KinematicNode*> const& node : tree.getNodes()) {
		kinematics::MotorValuesMap const& minVals = node.second->getMinValues();
		kinematics::MotorValuesMap const& maxVals = node.second->getMaxValues();
		kinematics::Motor2IntMap const& m2iMap = node.second->getMotor2IntMap();

		for (kinematics::Motor2Int const& m2i : m2iMap) {
			m_minValues(m2i.second) = minVals.at(m2i.first);
			m_maxValues(m2i.second) = maxVals.at(m2i.first);
		}
	}
}

ConstraintAngles::~ConstraintAngles() {
}

arma::colvec ConstraintAngles::getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const
{
	UNUSED(tree); // for these constraints we dont need the tree
	arma::colvec clippingError = arma::zeros(m_numMotors);

	for (int i(0); i < m_numMotors; ++i) {
		if (targetAngles(i) < m_minValues(i)) {
			clippingError(i) = m_minValues(i) - targetAngles(i);
		} else if (targetAngles(i) > m_maxValues(i)) {
			clippingError(i) = m_maxValues(i) - targetAngles(i);
		}
	}

	return clippingError;
}

}
