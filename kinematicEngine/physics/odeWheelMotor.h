/*
 * ODEWheelMotor.h
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#ifndef ODEWHEELMOTOR_H_
#define ODEWHEELMOTOR_H_

#include "odeHingeMotor.h"

namespace kinematicEngine {

class ODEWheelMotor : public ODEHingeMotor {
public:
	ODEWheelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double maxSpeed);

	virtual ~ODEWheelMotor();

	virtual void setDesiredValueAndSpeed(double targetValue, double targetSpeed) override;
	void setDesiredValue(double targetValue) override;
	void setDesiredSpeed(double targetSpeed) override;

	void simulatorCallback(double timeDelta) override;

private:

	void updateJointForces(double timeDelta);
};

}

#endif /* ODEWHEELMOTOR_H_ */
