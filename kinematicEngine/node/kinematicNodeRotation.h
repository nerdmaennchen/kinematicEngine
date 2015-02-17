/*
 * KinematicNodeRotation.h
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEROTATION_H_
#define KINEMATICNODEROTATION_H_

#include "utils/units.h"
#include "kinematicNode.h"

#include "kinematicEngine/physics/odeHingeMotor.h"

namespace kinematicEngine {

class KinematicNodeRotation: public KinematicNode {
public:
	KinematicNodeRotation();

	KinematicNodeRotation(
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

	virtual ~KinematicNodeRotation();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodeRotation* node = new KinematicNodeRotation();
		*node = *this;
		node->setParent(nullptr);
		node->m_children.clear();
		return node;
	}

	virtual bool isServo() const override {
		return true;
	}

	virtual void setValues(MotorValuesMap newValues) override;

	virtual bool isFixedNode() const
	{
		return false;
	}

	virtual kinematics::JacobianValues getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const;

	virtual kinematics::JacobianValues getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const;

	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const;

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame);

	virtual void setTorqueForODE(double torque);

protected:
	dJointID m_rotationJoint;

	double m_maxForce;
	double m_maxSpeed;

	ODEHingeMotor *m_hingeMotor;
};

}

#endif /* KINEMATICNODEROTATION_H_ */
