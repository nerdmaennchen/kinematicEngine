/*
 * kinematicEngineConstraintAngles.cpp
 *
 *  Created on: 08.01.2015
 *      Author: lutz
 */

#include <tools/kinematicEngine/constraints/kinematicEngineConstraintAngles.h>


KinematicEngineConstraintAngles::KinematicEngineConstraintAngles()
	: m_minValues(arma::zeros(1))
	, m_maxValues(arma::zeros(1))
	, m_numMotors(0)
{}

KinematicEngineConstraintAngles::KinematicEngineConstraintAngles(KinematicTree const& tree)
{
	m_numMotors = tree.getMotorCt();
	m_minValues = arma::zeros(m_numMotors);
	m_maxValues = arma::zeros(m_numMotors);

	arma::colvec motorIDsVec = arma::zeros(m_numMotors);
	for (int i(0); i < m_numMotors; ++i) {
		const KinematicNode* node = tree.getNode(tree.toExt(i));
		m_minValues(i) = Radian(Degree(node->getMinValue() * degrees)).value();
		m_maxValues(i) = Radian(Degree(node->getMaxValue() * degrees)).value();
		motorIDsVec(i) = tree.toExt(i);
	}

	std::cout.precision(3);
	std::cout << std::fixed << std::setw(7);
	motorIDsVec.t().raw_print(std::cout);
	m_minValues.t().raw_print(std::cout);
	m_maxValues.t().raw_print(std::cout);
}

KinematicEngineConstraintAngles::~KinematicEngineConstraintAngles() {
}

arma::colvec KinematicEngineConstraintAngles::getAngleErrorForConstraint(KinematicTree const& tree, arma::colvec const& targetAngles) const
{
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
