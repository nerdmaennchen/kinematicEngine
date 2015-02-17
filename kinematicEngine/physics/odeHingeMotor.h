/*
 * ODEMotor.h
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#ifndef ODEHINGEMOTOR_H_
#define ODEHINGEMOTOR_H_

#include "odeMotor.h"

namespace kinematicEngine {

class ODEHingeMotor : public ODEMotor {

public:
	ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed, double complianceSlope, double compliangeMargin);
	ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, double defaultAngle, double minAngle, double maxAngle, double maxSpeed);
	virtual ~ODEHingeMotor();

	virtual double getCurValue() const;
	virtual double getCurSpeed() const;
	virtual void setDesiredValueAndSpeed(double targetValue, double targetSpeed);
	virtual void setDesiredValue(double targetValue);
	virtual void setDesiredSpeed(double targetSpeed);

	virtual void   setValueOffset(double offset);
	virtual double getValueOffset();

	virtual void enableMotor(bool enabled);

	void setPD(double p, double d);

	/**
	 * called from the simulation environment before each simulation step
	 */
	virtual void simulatorCallback(double timeDelta);

protected:

	void updateJointForces(double timeDelta);

	double m_maxNewtonMeter;

	double m_defaultAngle;
	double m_zeroAngle;
	double m_minAngle;
	double m_maxAngle;

	double m_maxSpeed;

	double m_complianceSlope;
	double m_complianceMargin;

	double m_p, m_d;

	dJointFeedback m_jointFeedback;

	double m_maxSpeedToReachTarget;    // speed used to reach the target angle

	double m_targetAngle;

	double m_angleOffset;

	double removeOffset(double angle) const;
	double addOffset(double angle) const;
};

}

#endif /* ODEHINGEMOTOR_H_ */
