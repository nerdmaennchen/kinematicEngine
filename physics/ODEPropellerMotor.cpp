/*
 * ODEPropellerMotor.cpp
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#include "ODEPropellerMotor.h"
#include "../node/kinematicNode.h"

ODEPropellerMotor::ODEPropellerMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, RPM maxSpeed, double speedToForceFactor)
	: ODEWheelMotor(motorID, jointID, enviroment, node, maxNewtonmeter, maxSpeed)
	, m_speedToForceFactor(speedToForceFactor)
{
}

ODEPropellerMotor::~ODEPropellerMotor() {
}

void ODEPropellerMotor::simulatorCallback(Second timeDelta)
{
	::ODEWheelMotor::simulatorCallback(timeDelta);
	// calculate which force is generated
	RPM curSpeed = getCurSpeed();

	dReal rotationVec[3];
	dReal rotationPosition[3];

	double forceFactor = curSpeed.value() * m_speedToForceFactor;

	dJointGetHingeAxis(m_jointID, rotationVec);
	dJointGetHingeAnchor(m_jointID, rotationPosition);

	dBodyAddForceAtPos(m_kinematicNode->getODEBody(),
			rotationVec[0] * forceFactor,
			rotationVec[1] * forceFactor,
			rotationVec[2] * forceFactor,
			rotationPosition[0],
			rotationPosition[1],
			rotationPosition[2]);
}

