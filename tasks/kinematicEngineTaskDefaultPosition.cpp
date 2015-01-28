/*
 * kinematicEngineTaskDefaultPosition.cpp
 *
 *  Created on: 28.09.2014
 *      Author: lutz
 */

#include <tools/kinematicEngine/tasks/kinematicEngineTaskDefaultPosition.h>

KinematicEngineTaskDefaultPosition::KinematicEngineTaskDefaultPosition()
	: m_weight(1.)
	, m_speed(0.)
{
}

KinematicEngineTaskDefaultPosition::~KinematicEngineTaskDefaultPosition() {
}


void KinematicEngineTaskDefaultPosition::setDefaultValues(KinematicTree const& tree, std::map<MotorID, Degree> defaultValues) {
	uint motorCnt = tree.getMotorCt();
	m_defaultValues = arma::zeros(motorCnt);

	for (uint i = 0; i < motorCnt; ++i) {
		m_defaultValues(i) = Radian(defaultValues[tree.toExt(i)]).value();
	}
}


arma::colvec KinematicEngineTaskDefaultPosition::getErrors(KinematicTree const& tree) const
{
	uint motorCnt = tree.getMotorCt();
	std::map<MotorID, Degree> values;
	tree.getMotorValues(values);

	arma::colvec curValues = arma::zeros(motorCnt);

	for (uint i = 0; i < motorCnt; ++i) {
		curValues(i) = Radian(values[tree.toExt(i)]).value();
	}

	return (m_defaultValues - curValues) * m_weight;
}

