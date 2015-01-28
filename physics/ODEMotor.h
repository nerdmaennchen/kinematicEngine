/*
 * ODEMotor.h
 *
 *  Created on: 03.06.2014
 *      Author: lutz
 */

#ifndef ODEMOTOR_H_
#define ODEMOTOR_H_

#include <utils/units.h>
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
//		m_physicsEnvironment->addStepCallback(this);
		m_physicsEnvironment->addMotor(motorID, this);
	}

	virtual ~ODEMotor() {};

	virtual Degree getCurAngle() const = 0;
	virtual RPM getCurSpeed() const = 0;

	virtual void setDesiredAngleAndSpeed(Degree targetAngle, RPM targetSpeed) = 0;
	virtual void setDesiredAngle(Degree targetAngle) = 0;
	virtual void setDesiredSpeed(RPM targetSpeed) = 0;

	virtual void enableMotor(bool enabled) = 0;


	virtual void   setAngleOffset(Degree offset) = 0;
	virtual Degree getAngleOffset() = 0;

	virtual void simulatorCallback(Second timeDelta) = 0;

protected:
	dJointID m_jointID;
	KinematicNode *m_kinematicNode;

private:
	PhysicsEnvironment *m_physicsEnvironment;
	MotorID m_motorID;
};

#endif /* ODEMOTOR_H_ */
