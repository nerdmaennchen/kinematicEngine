/*
 * ODEPropellerMotor.cpp
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#include "ODEPropellerMotor.h"
#include "../node/kinematicNode.h"

ODEPropellerMotor::ODEPropellerMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double maxSpeed, double speedToForceFactor)
	: ODEWheelMotor(motorID, jointID, enviroment, node, maxNewtonmeter, maxSpeed)
	, m_speedToForceFactor(speedToForceFactor)
{
}

ODEPropellerMotor::~ODEPropellerMotor() {
}

void ODEPropellerMotor::simulatorCallback(double timeDelta)
{
	::ODEWheelMotor::simulatorCallback(timeDelta);
	// calculate which force is generated
	double curSpeed = getCurSpeed();

	dReal rotationVec[3];
	dReal rotationPosition[3];

	double forceFactor = curSpeed * m_speedToForceFactor;

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

