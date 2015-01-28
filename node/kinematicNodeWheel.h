/*
 * KinematicNodeWheel.h
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEWHEEL_H_
#define KINEMATICNODEWHEEL_H_

#include "kinematicNodeRotation.h"
#include "../physics/ODEWheelMotor.h"
#include "utils/units.h"

class KinematicNodeWheel : public KinematicNodeRotation {
public:
	KinematicNodeWheel();

	KinematicNodeWheel(MotorID id,
			KinematicNode *parent,
			std::string name,
			double maxForce,
			RPM maxSpeed,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ);

	virtual ~KinematicNodeWheel();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodeWheel* node = new KinematicNodeWheel();
		*node = *this;
		node->setParent(nullptr);
		node->m_children.clear();
		return node;
	}

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame);

private:
	dJointID m_rotationJoint;
	ODEWheelMotor *m_motor;

};

#endif /* KINEMATICNODEWHEEL_H_ */
