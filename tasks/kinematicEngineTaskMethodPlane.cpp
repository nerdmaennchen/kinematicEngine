/*
 * kinematicEngineTaskMethodPlane.cpp
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#include <tools/kinematicEngine/tasks/kinematicEngineTaskMethodPlane.h>

KinematicEngineTaskMethodPlane::KinematicEngineTaskMethodPlane()
	: m_transform(arma::zeros(1, 3))
	, m_target(arma::zeros(1))
{
}

KinematicEngineTaskMethodPlane::~KinematicEngineTaskMethodPlane() {
}

arma::mat const& KinematicEngineTaskMethodPlane::getTransform() const
{
	return m_transform;
}

arma::colvec const& KinematicEngineTaskMethodPlane::getTarget() const
{
	return m_target;
}

void KinematicEngineTaskMethodPlane::setPlaneParams(arma::colvec3 normalVec, arma::colvec3 supportPoint)
{
	normalVec = normalVec * 1. / arma::norm(normalVec, 2);

	m_transform = arma::zeros(1, 3);
	m_transform.row(0) = normalVec.t();

	m_target = m_transform * supportPoint;
}
