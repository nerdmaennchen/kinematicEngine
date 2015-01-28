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

ODEWheelMotor::ODEWheelMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, RPM maxSpeed)
	: ODEHingeMotor(motorID, jointID, enviroment, node, maxNewtonmeter, 0 * degrees, -360 * degrees, 360 * degrees, maxSpeed)
{

}

void ODEWheelMotor::updateJointForces(Second timeDelta)
{
	dJointSetHingeParam(m_jointID, dParamVel, m_maxSpeedToReachTarget.value() * 2 * M_PI / 60.);
}

void ODEWheelMotor::setDesiredAngleAndSpeed(Degree targetAngle, RPM targetSpeed)
{
	m_maxSpeedToReachTarget = targetSpeed;
}

void ODEWheelMotor::setDesiredAngle(Degree targetAngle)
{
	// nothing to do
}

void ODEWheelMotor::setDesiredSpeed(RPM targetSpeed)
{
	m_maxSpeedToReachTarget = targetSpeed;
}

void ODEWheelMotor::simulatorCallback(Second timeDelta)
{
	updateJointForces(timeDelta);
}
