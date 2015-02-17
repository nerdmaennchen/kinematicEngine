/*
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#include "taskMethodPoint.h"

namespace kinematicEngine {

TaskMethodPoint::TaskMethodPoint()
	: TaskMethod()
	, m_target(arma::zeros(3))
	, m_transform(arma::eye(3, 3))
{
}

TaskMethodPoint::TaskMethodPoint(arma::colvec3 target)
	: TaskMethod()
	, m_target(target)
	, m_transform(arma::eye(3, 3))
{
}

TaskMethodPoint::~TaskMethodPoint() {
}

void TaskMethodPoint::setTarget(arma::colvec3 target)
{
	m_target = target;
}

arma::mat const& TaskMethodPoint::getTransform() const
{
	return m_transform;
}

arma::colvec const& TaskMethodPoint::getTarget() const
{
	return m_target;
}

}
