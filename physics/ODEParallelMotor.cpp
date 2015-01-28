/*
 * ODEParallelMotor.cpp
 *
 *  Created on: 03.06.2014
 *      Author: lutz
 */

#include "ODEParallelMotor.h"

#include <debug.h>

ODEParallelMotor::ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed, Degree complianceSlope, Degree compliangeMargin)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, defaultAngle, minAngle, maxAngle, maxSpeed, complianceSlope, compliangeMargin)
{}

ODEParallelMotor::ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, defaultAngle, minAngle, maxAngle, maxSpeed)
{}

ODEParallelMotor::~ODEParallelMotor() {
}
