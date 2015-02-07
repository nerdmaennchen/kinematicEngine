/*
 * ODEMotor.cpp
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#include "ODEHingeMotor.h"

#include "../node/kinematicNode.h"
#include "utils/utils.h"

//#define DEFAULT_MP 0.
//#define DEFAULT_MD 0.00

#define DEFAULT_MP 10.
#define DEFAULT_MD 0.00005

ODEHingeMotor::ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed, double complianceSlope, double compliangeMargin)
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
	, m_maxSpeedToReachTarget(0)
	, m_targetAngle(0)
	, m_angleOffset(dJointGetHingeAngle(m_jointID))
{
	if (dJointTypeHinge != dJointGetType(m_jointID))
	{
		std::cerr << "Cannot attach ODEHingeMotor to non-hinge joint! Joint type is: " << dJointGetType(m_jointID) << std::endl;
	} else {
		dJointSetHingeParam(m_jointID, dParamFMax, m_maxNewtonMeter);
		dJointSetFeedback(m_jointID, &m_jointFeedback);

		m_zeroAngle = m_defaultAngle;
		setValueOffset(0.);

		setDesiredValueAndSpeed(0., m_maxSpeedToReachTarget);
	}
}

ODEHingeMotor::ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed)
	: ODEMotor(motorID, jointID, enviroment, node)
	, m_maxNewtonMeter(maxNewtonmeter)
	, m_defaultAngle(defaultAngle)
	, m_minAngle(minAngle)
	, m_maxAngle(maxAngle)
	, m_maxSpeed(maxSpeed)
	, m_complianceSlope(0)
	, m_complianceMargin(0)
	, m_p(DEFAULT_MP)
	, m_d(DEFAULT_MD)
	, m_maxSpeedToReachTarget(0)
	, m_targetAngle(0)
	, m_angleOffset(0)
{
	if (dJointTypeHinge != dJointGetType(m_jointID))
	{
		std::cerr << "Cannot attach ODEHingeMotor to non-hinge joint! Joint type is: " << dJointGetType(m_jointID) << std::endl;
	} else {
		dJointSetHingeParam(m_jointID, dParamFMax, m_maxNewtonMeter);
		dJointSetFeedback(m_jointID, &m_jointFeedback);

		m_zeroAngle = m_defaultAngle;
		setValueOffset(0);
		setDesiredValueAndSpeed(0, m_maxSpeedToReachTarget);
	}
}

ODEHingeMotor::~ODEHingeMotor()
{
}

double ODEHingeMotor::getCurValue() const
{
	double rad = dJointGetHingeAngle(m_jointID);
	return addOffset(rad);
}


double ODEHingeMotor::getCurSpeed() const
{
	return dJointGetHingeAngleRate(m_jointID);
}

void ODEHingeMotor::setValueOffset(double offset)
{
	m_angleOffset = m_zeroAngle + offset;

	// update min and max values of this joint
	dJointSetHingeParam(m_jointID, dParamHiStop, removeOffset(m_maxAngle));
	dJointSetHingeParam(m_jointID, dParamLoStop, removeOffset(m_minAngle));
}

double ODEHingeMotor::getValueOffset()
{
	return m_angleOffset;
}

void ODEHingeMotor::setPD(double p, double d)
{
	m_p = p;
	m_d = d;
}

void ODEHingeMotor::updateJointForces(double timeDelta)
{
	double curAngle = getCurValue();

	double pDiff = utils::normalize((m_targetAngle - curAngle) * radians).value() * m_p;
	const double speedHelper = std::abs(m_maxSpeedToReachTarget);
	double vel = utils::limited(pDiff, -speedHelper, speedHelper);
	dJointSetHingeParam(m_jointID, dParamVel, vel);
}

void ODEHingeMotor::setDesiredValueAndSpeed(double targetAngle, double targetSpeed)
{
	m_targetAngle = utils::limited(targetAngle, m_minAngle, m_maxAngle);
	m_maxSpeedToReachTarget = std::min(targetSpeed, m_maxSpeed);
//	updateJointForces(0);
}

void ODEHingeMotor::setDesiredValue(double targetAngle)
{
	m_targetAngle = utils::limited(targetAngle, m_minAngle, m_maxAngle);
//	updateJointForces(0);
}

void ODEHingeMotor::setDesiredSpeed(double targetSpeed)
{
	m_maxSpeedToReachTarget = std::min(targetSpeed, m_maxSpeed);
//	updateJointForces(0);
}

void ODEHingeMotor::simulatorCallback(double timeDelta)
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

double ODEHingeMotor::removeOffset(double angle) const
{
	return angle - m_angleOffset;
}

double ODEHingeMotor::addOffset(double angle) const
{
	return angle + m_angleOffset;
}
