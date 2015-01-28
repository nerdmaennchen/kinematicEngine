/*
 * ODEMotor.cpp
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#include "ODEHingeMotor.h"

#include "../node/kinematicNode.h"
#include <utils/math/Math.h>
#include <debug.h>

//#define DEFAULT_MP 0.
//#define DEFAULT_MD 0.00

#define DEFAULT_MP 10.
#define DEFAULT_MD 0.00005

ODEHingeMotor::ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed, Degree complianceSlope, Degree compliangeMargin)
	: ODEMotor(motorID, jointID, enviroment, node)
	, m_maxNewtonMeter(maxNewtonmeter)
	, m_defaultAngle(defaultAngle)
	, m_minAngle(minAngle)
	, m_maxAngle(maxAngle)
	, m_maxSpeed(maxSpeed)
	, m_complianceSlope(complianceSlope)
	, m_complianceMargin(compliangeMargin)
	, m_p(DEFAULT_MP)
	, m_d(DEFAULT_MD)
	, m_maxSpeedToReachTarget(0 * rounds_per_minute)
	, m_targetAngle(0 * degrees)
	, m_angleOffset(dJointGetHingeAngle(m_jointID) * degrees)
{
	if (dJointTypeHinge != dJointGetType(m_jointID))
	{
		ERROR("Cannot attach ODEHingeMotor to non-hinge joint! Joint type is: %d", dJointGetType(m_jointID));
	} else {
		dJointSetHingeParam(m_jointID, dParamFMax, m_maxNewtonMeter);
		dJointSetFeedback(m_jointID, &m_jointFeedback);

		m_zeroAngle = m_defaultAngle;
		setAngleOffset(0 * degrees);

		setDesiredAngleAndSpeed(0 * degrees, m_maxSpeedToReachTarget);
	}
}

ODEHingeMotor::ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed)
	: ODEMotor(motorID, jointID, enviroment, node)
	, m_maxNewtonMeter(maxNewtonmeter)
	, m_defaultAngle(defaultAngle)
	, m_minAngle(minAngle)
	, m_maxAngle(maxAngle)
	, m_maxSpeed(maxSpeed)
	, m_complianceSlope(0 * degrees)
	, m_complianceMargin(0 * degrees)
	, m_p(DEFAULT_MP)
	, m_d(DEFAULT_MD)
	, m_maxSpeedToReachTarget(0 * rounds_per_minute)
	, m_targetAngle(0 * degrees)
	, m_angleOffset(0 * degrees)
{
	if (dJointTypeHinge != dJointGetType(m_jointID))
	{
		ERROR("Cannot attach ODEHingeMotor to non-hinge joint! Joint type is: %d", dJointGetType(m_jointID));
	} else {
		dJointSetHingeParam(m_jointID, dParamFMax, m_maxNewtonMeter);
		dJointSetFeedback(m_jointID, &m_jointFeedback);

		m_zeroAngle = m_defaultAngle;
		setAngleOffset(0 * degrees);
		setDesiredAngleAndSpeed(0 * degrees, m_maxSpeedToReachTarget);
	}
}

ODEHingeMotor::~ODEHingeMotor()
{
}

Degree ODEHingeMotor::getCurAngle() const
{
	double rad = dJointGetHingeAngle(m_jointID);
	return addOffset(Degree(rad * radians));
}


RPM ODEHingeMotor::getCurSpeed() const
{
	double radPS = dJointGetHingeAngleRate(m_jointID);
	return RPM(60. * radPS / (2. * M_PI) * rounds_per_minute);
}

void ODEHingeMotor::setAngleOffset(Degree offset)
{
	m_angleOffset = m_zeroAngle + offset;

	// update min and max values of this joint
	dJointSetHingeParam(m_jointID, dParamHiStop, Radian(removeOffset(m_maxAngle)).value());
	dJointSetHingeParam(m_jointID, dParamLoStop, Radian(removeOffset(m_minAngle)).value());
}

Degree ODEHingeMotor::getAngleOffset()
{
	return m_angleOffset;
}

void ODEHingeMotor::setPD(double p, double d)
{
	m_p = p;
	m_d = d;
}

void ODEHingeMotor::updateJointForces(Second timeDelta)
{
	Degree curAngle = getCurAngle();
//	RPM curSpeed = getCurSpeed();

	double pDiff = Math::normalize(Radian(m_targetAngle - curAngle)).value() * DEFAULT_MP;

//	double dDiff = curSpeed.value() * 2. * M_PI / 60.;

//	double torque = m_p / timeDelta.value() * pDiff - m_d / timeDelta.value() * dDiff;
//	torque = Math::limited(torque, -m_maxNewtonMeter, m_maxNewtonMeter);
//	dJointAddHingeTorque(m_jointID, torque);

	const double speedHelper = m_maxSpeedToReachTarget.value() * (2. * M_PI) / 60.;
	double vel = Math::limited(pDiff, -speedHelper, speedHelper);
	dJointSetHingeParam(m_jointID, dParamVel, vel);

//	INFO("%d \tangle % 6.2f,\tangleRate % 6.2f,\ttorque % 6.2f\tpDiff % 6.2f\tdDiff % 6.2f\tangleOff: % 6.2f\ttarget: % 6.2f",
//			this->m_kinematicNode->getID(),
//			curAngle.value(),
//			curSpeed.value(),
//			torque,
//			pDiff,
//			dDiff,
//			m_angleOffset.value(),
//			m_targetAngle.value());

//	dVector3 v1, v2;
//
//	memset(v1, 0, sizeof(v1));
//	memset(v1, 0, sizeof(v2));
//
//	dBodyID body1, body2;
//
//	body1 = dJointGetBody(m_jointID, 0);
//	body2 = dJointGetBody(m_jointID, 1);
//
//	if (0 != body1) {
//		dBodyVectorFromWorld(body1, m_jointFeedback.t1[0], m_jointFeedback.t1[1], m_jointFeedback.t1[2], v1);
//	}
//
//	if (0 != body2) {
//		dBodyVectorFromWorld(body2, m_jointFeedback.t2[0], m_jointFeedback.t2[1], m_jointFeedback.t2[2], v2);
//	}

//	printf("\t%d Torque\n\tb1:% 6.2f % 6.2f % 6.2f\n\tb2:% 6.2f % 6.2f % 6.2f\n",
//			this->m_kinematicNode->getID(),
//			v1[0], v1[1], v1[2],
//			v2[0], v2[1], v2[2]);

//	memset(v1, 0, sizeof(v1));
//	memset(v1, 0, sizeof(v2));
//
//	if (0 != body1) {
//		dBodyVectorFromWorld(body1, m_jointFeedback.f1[0], m_jointFeedback.f1[1], m_jointFeedback.f1[2], v1);
//	}
//	if (0 != body2) {
//		dBodyVectorFromWorld(body2, m_jointFeedback.f2[0], m_jointFeedback.f2[1], m_jointFeedback.f2[2], v2);
//	}
//	printf("\t%d Force\n\tb1:\t% 6.2f % 6.2f % 6.2f\n\tb2:% 6.2f % 6.2f % 6.2f\n",
//			this->m_kinematicNode->getID(),
//			v1[0], v1[1], v1[2],
//			v2[0], v2[1], v2[2]);
}

void ODEHingeMotor::setDesiredAngleAndSpeed(Degree targetAngle, RPM targetSpeed)
{
	m_targetAngle = Math::limited(targetAngle, m_minAngle, m_maxAngle);
	m_maxSpeedToReachTarget = std::min(targetSpeed, m_maxSpeed);
	updateJointForces(0 * seconds);
}

void ODEHingeMotor::setDesiredAngle(Degree targetAngle)
{
	m_targetAngle = Math::limited(targetAngle, m_minAngle, m_maxAngle);
	updateJointForces(0 * seconds);
}

void ODEHingeMotor::setDesiredSpeed(RPM targetSpeed)
{
	m_maxSpeedToReachTarget = std::min(targetSpeed, m_maxSpeed);
	updateJointForces(0 * seconds);
}

void ODEHingeMotor::simulatorCallback(Second timeDelta)
{
	updateJointForces(timeDelta);
}


void ODEHingeMotor::enableMotor(bool enabled)
{
	if (enabled) {
		dJointSetHingeParam(m_jointID, dParamFMax, m_maxNewtonMeter);
	} else {
		dJointSetHingeParam(m_jointID, dParamFMax, 0);
	}
}

Degree ODEHingeMotor::removeOffset(Degree angle) const
{
	return angle - m_angleOffset;
}

Degree ODEHingeMotor::addOffset(Degree angle) const
{
	return angle + m_angleOffset;
}
