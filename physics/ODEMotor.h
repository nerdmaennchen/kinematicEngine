/*
 * ODEMotor.h
 *
 *  Created on: 03.06.2014
 *      Author: lutz
 */

#ifndef ODEMOTOR_H_
#define ODEMOTOR_H_

#include "utils/units.h"
#include <ode/ode.h>

#include "physicsEnvironment.h"

class KinematicNode;

class ODEMotor : public PhysicsEnvironmentStepCallback {
public:
	ODEMotor(MotorID motorID, dJointID jointID, PhysicsEnvironment *enviroment, KinematicNode* node)
	: m_jointID(jointID)
	, m_kinematicNode(node)
	, m_physicsEnvironment(enviroment)
	, m_motorID(motorID)
	{
		m_physicsEnvironment->addMotor(motorID, this);
	}

	virtual ~ODEMotor() {};

	virtual double getCurValue() const = 0;
	virtual double getCurSpeed() const = 0;

	virtual void setDesiredValueAndSpeed(double targetValue, double targetSpeed) = 0;
	virtual void setDesiredValue(double targetValue) = 0;
	virtual void setDesiredSpeed(double targetSpeed) = 0;

	virtual void enableMotor(bool enabled) = 0;


	virtual void   setValueOffset(double offset) = 0;
	virtual double getValueOffset() = 0;

	virtual void simulatorCallback(double timeDelta) = 0;

protected:
	dJointID m_jointID;
	KinematicNode *m_kinematicNode;

private:
	PhysicsEnvironment *m_physicsEnvironment;
	MotorID m_motorID;
};

#endif /* ODEMOTOR_H_ */
