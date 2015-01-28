/*
 * kinematicNodeFixed.h
 *
 *  Created on: 23.09.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEFIXED_H_
#define KINEMATICNODEFIXED_H_

#include "kinematicNodeDummy.h"

class KinematicNodeFixed : public KinematicNodeDummy {
public:
	KinematicNodeFixed();

	KinematicNodeFixed(
			MotorID id,
			KinematicNode *parent,
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ);

	virtual ~KinematicNodeFixed();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodeFixed* node = new KinematicNodeFixed();
		*node = *this;
		node->setParent(nullptr);
		node->m_children.clear();
		return node;
	}

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 coordinateFrame);

private:
	dJointID m_fixedJoint;
};

#endif /* KINEMATICNODEFIXED_H_ */
