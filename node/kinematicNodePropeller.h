/*
 * KinematicNodeWheel.h
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEPROPELLER_H_
#define KINEMATICNODEPROPELLER_H_

#include "kinematicNodeWheel.h"
#include "../physics/ODEPropellerMotor.h"
#include "utils/units.h"

class KinematicNodePropeller : public KinematicNodeWheel {
public:
	KinematicNodePropeller();

	KinematicNodePropeller(MotorID id,
			KinematicNode *parent,
			std::string name,
			double maxForce,
			RPM maxSpeed,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			double speedToForceFactor);

	virtual ~KinematicNodePropeller();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodePropeller* node = new KinematicNodePropeller();
		*node = *this;
		node->setParent(nullptr);
		node->m_children.clear();
		return node;
	}

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame);

private:
	dJointID m_rotationJoint;
	ODEPropellerMotor *m_motor;
	double m_speedToForceFactor;

};

#endif /* KINEMATICNODEPROPELLER_H_ */
