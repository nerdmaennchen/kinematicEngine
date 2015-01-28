/*
 * kinematicEngineTaskMethodLine.cpp
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#include "kinematicEngineTaskMethodLine.h"

KinematicEngineTaskMethodLine::KinematicEngineTaskMethodLine()
	: m_transform(arma::zeros(2, 3))
	, m_target(arma::zeros(2))
{
}

KinematicEngineTaskMethodLine::~KinematicEngineTaskMethodLine() {
}

arma::mat const& KinematicEngineTaskMethodLine::getTransform() const
{
	return m_transform;
}

arma::colvec const& KinematicEngineTaskMethodLine::getTarget() const
{
	return m_target;
}

void KinematicEngineTaskMethodLine::setLineParams(arma::colvec3 direction, arma::colvec3 position)
{
	const arma::colvec3 XAxis = arma::colvec({1, 0, 0});
	const arma::colvec3 YAxis = arma::colvec({0, 1, 0});
	const arma::colvec3 ZAxis = arma::colvec({0, 0, 1});

	double scoreX = std::abs(arma::dot(XAxis, direction));
	double scoreY = std::abs(arma::dot(YAxis, direction));
	double scoreZ = std::abs(arma::dot(ZAxis, direction));

	arma::colvec3 helperDir = ZAxis;

	if (scoreZ <= scoreX && scoreZ <= scoreY) {
		helperDir = ZAxis;
	} else if (scoreY <= scoreX && scoreY <= scoreZ) {
		helperDir = YAxis;
	} else {
		helperDir = XAxis;
	}

	arma::colvec helperZ = arma::cross(direction, helperDir);
	arma::colvec helperY = arma::cross(helperZ, direction);

	direction = direction * 1. / arma::norm(direction, 2);
	helperY = helperY * 1. / arma::norm(helperY, 2);
	helperZ = helperZ * 1. / arma::norm(helperZ, 2);

	m_transform = arma::zeros(2, 3);
	m_transform.row(0) = helperZ.t();
	m_transform.row(1) = helperY.t();

	m_target = m_transform * position;
}

