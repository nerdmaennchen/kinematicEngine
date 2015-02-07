/*
 * ODEParallelMotor.cpp
 *
 *  Created on: 03.06.2014
 *      Author: lutz
 */

#include "ODEParallelMotor.h"

ODEParallelMotor::ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed, double complianceSlope, double compliangeMargin)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, defaultAngle, minAngle, maxAngle, maxSpeed, complianceSlope, compliangeMargin)
{}

ODEParallelMotor::ODEParallelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, defaultAngle, minAngle, maxAngle, maxSpeed)
{}

ODEParallelMotor::~ODEParallelMotor() {
}
