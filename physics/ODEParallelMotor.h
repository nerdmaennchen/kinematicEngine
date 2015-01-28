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
	ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed, Degree complianceSlope, Degree compliangeMargin);
	ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed);
	virtual ~ODEParallelMotor();
};

#endif /* ODEPARALLELMOTOR_H_ */
