/*
 * ODEMotor.h
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#ifndef ODEHINGEMOTOR_H_
#define ODEHINGEMOTOR_H_

#include "ODEMotor.h"

class ODEHingeMotor : public ODEMotor {

public:
	ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed, Degree complianceSlope, Degree compliangeMargin);
	ODEHingeMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node, double maxNewtonmeter, Degree defaultAngle, Degree minAngle, Degree maxAngle, RPM maxSpeed);
	virtual ~ODEHingeMotor();

	virtual Degree getCurAngle() const;
	virtual RPM getCurSpeed() const;
	virtual void setDesiredAngleAndSpeed(Degree targetAngle, RPM targetSpeed);
	virtual void setDesiredAngle(Degree targetAngle);
	virtual void setDesiredSpeed(RPM targetSpeed);

	virtual void   setAngleOffset(Degree offset);
	virtual Degree getAngleOffset();

	virtual void enableMotor(bool enabled);

	void setPD(double p, double d);

	/**
	 * called from the simulation environment before each simulation step
	 */
	virtual void simulatorCallback(Second timeDelta);

protected:

	void updateJointForces(Second timeDelta);

	double m_maxNewtonMeter;

	Degree m_defaultAngle;
	Degree m_zeroAngle;
	Degree m_minAngle;
	Degree m_maxAngle;

	RPM m_maxSpeed;

	Degree m_complianceSlope;
	Degree m_complianceMargin;

	double m_p, m_d;

	dJointFeedback m_jointFeedback;

	RPM m_maxSpeedToReachTarget;    // speed used to reach the target angle

	Degree m_targetAngle;

	Degree m_angleOffset;

	Degree removeOffset(Degree angle) const;
	Degree addOffset(Degree angle) const;
};

#endif /* ODEHINGEMOTOR_H_ */
