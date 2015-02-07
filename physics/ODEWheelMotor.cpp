/*
 * ODEWheelMotor.cpp
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#include "ODEWheelMotor.h"


ODEWheelMotor::~ODEWheelMotor() {
	// TODO Auto-generated destructor stub
}

ODEWheelMotor::ODEWheelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double maxSpeed)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, 0, -2 * M_PI, 2 * M_PI, maxSpeed)
{

}

void ODEWheelMotor::updateJointForces(double timeDelta)
{
	dJointSetHingeParam(m_jointID, dParamVel, m_maxSpeedToReachTarget);
}

void ODEWheelMotor::setDesiredValueAndSpeed(double targetAngle, double targetSpeed)
{
	m_maxSpeedToReachTarget = targetSpeed;
}

void ODEWheelMotor::setDesiredValue(double targetAngle)
{
	// nothing to do
}

void ODEWheelMotor::setDesiredSpeed(double targetSpeed)
{
	m_maxSpeedToReachTarget = targetSpeed;
}

void ODEWheelMotor::simulatorCallback(double timeDelta)
{
	updateJointForces(timeDelta);
}
