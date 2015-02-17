/*
 * kinematicMass.cpp
 *
 *  Created on: 16.02.2014
 *      Author: lutz
 */

#include "kinematicMass.h"

namespace kinematicEngine {

KinematicMass::KinematicMass():
	m_position(arma::zeros(3)),
	m_massGrams(0.),
	m_name("")
{
}

KinematicMass::KinematicMass(arma::colvec3 position, double massGrams, std::string name):
	m_position(position),
	m_massGrams(massGrams),
	m_name(name)
{
}

KinematicMass::~KinematicMass() {
}


dMass KinematicMass::getODEMass(arma::mat44 coordinateFrame) const
{
	dMass mass;

	arma::colvec4 helper = arma::zeros(4);
	helper(3) = 1.;
	helper.rows(0, 2) = m_position;
	helper = coordinateFrame * helper;

	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, m_massGrams / 1000., 1);
	dMassTranslate(&mass, helper(0), helper(1), helper(2));

	return mass;
}

}
