/*
 * kinematicMass.h
 *
 *  Created on: 16.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICMASS_H_
#define KINEMATICMASS_H_

#include <armadillo>
#include "utils/units.h"
#include <string>
#include <ode/ode.h>

class KinematicMass {
public:
	KinematicMass();
	KinematicMass(arma::colvec3 position, double massGrams, std::string name);
	virtual ~KinematicMass();

	arma::colvec3 m_position;
	double m_massGrams;

	KinematicMass operator+(KinematicMass const& rhs) const {
		KinematicMass ret;
		ret.m_position = this->m_position * this->m_massGrams + rhs.m_position * rhs.m_massGrams;
		ret.m_massGrams = this->m_massGrams + rhs.m_massGrams;

		if (ret.m_massGrams != 0.) {
			ret.m_position *= 1. / ret.m_massGrams;
		}

		return ret;
	}

	KinematicMass& operator+=(KinematicMass const& rhs) {
		this->m_position *= this->m_massGrams;
		this->m_position += rhs.m_massGrams * rhs.m_position;

		this->m_massGrams += rhs.m_massGrams;

		if (this->m_massGrams != 0.) {
			this->m_position *= 1. / this->m_massGrams;
		}
		return *this;
	}

	KinematicMass mean(KinematicMass const& other) {
		KinematicMass ret;
		ret.m_massGrams = this->m_massGrams + other.m_massGrams;
		ret.m_position = (this->m_position * this->m_massGrams + other.m_position * other.m_massGrams) / ret.m_massGrams;
		return ret;
	}

	/**
	 * performs the homogenious transformation M * m_position
	 * as if m_position is a homogenious vector
	 * @param transform
	 */
	inline void applyTransformation(arma::mat44 transform) {
		arma::colvec4 helper = arma::ones(4);
		helper.rows(0, 2) = m_position;
		helper = transform * helper;
		m_position = helper.rows(0, 2);
	}

	std::string m_name;

	dMass getODEMass(arma::mat44 coordinateFrame);
};

#endif /* KINEMATICMASS_H_ */
