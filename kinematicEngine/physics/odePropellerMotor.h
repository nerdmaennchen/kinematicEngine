/*
 * ODEPropellerMotor.h
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#ifndef ODEPROPELLERMOTOR_H_
#define ODEPROPELLERMOTOR_H_

#include "odeWheelMotor.h"

namespace kinematicEngine {

class ODEPropellerMotor : public ODEWheelMotor {
public:
	ODEPropellerMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double maxSpeed, double speedToForceFactor);
	virtual ~ODEPropellerMotor();

	virtual void simulatorCallback(double timeDelta);

private:

	void updateJointForces(double timeDelta);

protected:
	double m_speedToForceFactor;
};

}

#endif /* ODEPROPELLERMOTOR_H_ */
