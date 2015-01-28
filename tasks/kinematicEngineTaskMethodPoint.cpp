/*
 * kinematicEngineTaskMethodPoint.cpp
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#include <tools/kinematicEngine/tasks/kinematicEngineTaskMethodPoint.h>

KinematicEngineTaskMethodPoint::KinematicEngineTaskMethodPoint()
	: KinematicEngineTaskMethod()
	, m_target(arma::zeros(3))
	, m_transform(arma::eye(3, 3))
{
}

KinematicEngineTaskMethodPoint::KinematicEngineTaskMethodPoint(arma::colvec3 target)
	: KinematicEngineTaskMethod()
	, m_target(target)
	, m_transform(arma::eye(3, 3))
{
}

KinematicEngineTaskMethodPoint::~KinematicEngineTaskMethodPoint() {
}

void KinematicEngineTaskMethodPoint::setTarget(arma::colvec3 target)
{
	m_target = target;
}

arma::mat const& KinematicEngineTaskMethodPoint::getTransform() const
{
	return m_transform;
}

arma::colvec const& KinematicEngineTaskMethodPoint::getTarget() const
{
	return m_target;
}

