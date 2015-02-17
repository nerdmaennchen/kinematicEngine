/*
 * kinematicNodeDummy.h
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEDUMMY_H_
#define KINEMATICNODEDUMMY_H_

#include "kinematicNode.h"
#include "kinematicEngine/utils/homogeniousTransform.h"

namespace kinematicEngine {

class KinematicNodeDummy : public KinematicNode {
public:
	KinematicNodeDummy();

	KinematicNodeDummy(
			KinematicNode *parent,
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ);

	virtual ~KinematicNodeDummy();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodeDummy* node = new KinematicNodeDummy();
		*node = *this;
		node->setParent(nullptr);
		node->m_children.clear();
		return node;
	}

	virtual bool isFixedNode() const
	{
		return true;
	}

	virtual kinematics::JacobianValues getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const override;

	virtual kinematics::JacobianValues getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const override;

	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const;

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 coordinateFrame);

	/**
	 * a dummy node must not possess a body and thus a mass.
	 * This function shall be overridden by any dummy node to pass its attached masses to its parent
	 * @param coordinateFrame parents coordinate frame
	 * @return
	 */
	virtual dMass getODEMassToAttachToParent(arma::mat44 coordinateFrame) const;

	virtual dBodyID getODEBody() const ;

	virtual void attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

	virtual void setTorqueForODE(double torque);
};

}

#endif
