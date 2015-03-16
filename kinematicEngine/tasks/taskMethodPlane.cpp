/*
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#include "taskMethodPlane.h"

namespace kinematicEngine {

TaskMethodPlane::TaskMethodPlane()
	: m_transform(arma::zeros(1, 3))
	, m_target(arma::zeros(1))
{
}

TaskMethodPlane::~TaskMethodPlane() {
}

arma::mat const& TaskMethodPlane::getTransform() const
{
	return m_transform;
}

arma::colvec const& TaskMethodPlane::getTarget() const
{
	return m_target;
}

void TaskMethodPlane::setPlaneParams(arma::colvec3 normalVec, arma::colvec3 supportPoint)
{
	const double normLen = arma::norm(normalVec, 2);
	if (normLen > 0.000000001) {
		normalVec = normalVec * 1. / normLen;
	} else {
		normalVec = arma::colvec({0, 0, 1});
	}

	m_transform = arma::zeros(1, 3);
	m_transform.row(0) = normalVec.t();

	m_target = m_transform * supportPoint;
}

}
