/*
 * ODEParallelMotor.h
 *
 *  Created on: 03.06.2014
 *      Author: lutz
 */

#ifndef ODEPARALLELMOTOR_H_
#define ODEPARALLELMOTOR_H_

#include "ODEHingeMotor.h"

class ODEParallelMotor : public ODEHingeMotor {
public:
	ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed, double complianceSlope, double compliangeMargin);
	ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed);
	virtual ~ODEParallelMotor();
};

#endif /* ODEPARALLELMOTOR_H_ */
