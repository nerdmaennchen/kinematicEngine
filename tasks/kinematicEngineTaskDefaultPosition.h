/*
 * kinematicEngineTaskDefaultPosition.h
 *
 *  Created on: 28.09.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKDEFAULTPOSITION_H_
#define KINEMATICENGINETASKDEFAULTPOSITION_H_

#include "../kinematicTree.h"

class KinematicEngineTaskDefaultPosition {
public:
	KinematicEngineTaskDefaultPosition();

	void setDefaultValues(KinematicTree const& tree, std::map<MotorID, Degree> defaultValues);

	virtual ~KinematicEngineTaskDefaultPosition();

	arma::colvec getDefaultValues() const {
		return m_defaultValues;
	}

	arma::colvec getErrors(KinematicTree const& tree) const;

	void setWeight(double newWeight) {
		m_weight = newWeight;
	}

	void setSpeed(double speed) {
		m_speed = speed;
	}

	double getWeight() {
		return m_weight;
	}

	double getSpeed() const {
		return m_speed;
	}

private:

	double m_weight;

	double m_speed;

	arma::colvec m_defaultValues;
};

#endif /* KINEMATICENGINETASKDEFAULTPOSITION_H_ */
