/*
 * ODEWheelMotor.h
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#ifndef ODEWHEELMOTOR_H_
#define ODEWHEELMOTOR_H_

#include "ODEHingeMotor.h"

class ODEWheelMotor : public ODEHingeMotor {
public:
	ODEWheelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, RPM maxSpeed);

	virtual ~ODEWheelMotor();


	virtual void setDesiredAngleAndSpeed(Degree targetAngle, RPM targetSpeed);
	virtual void setDesiredAngle(Degree targetAngle);
	virtual void setDesiredSpeed(RPM targetSpeed);

	virtual void simulatorCallback(Second timeDelta);

private:

	void updateJointForces(Second timeDelta);
};

#endif /* ODEWHEELMOTOR_H_ */
