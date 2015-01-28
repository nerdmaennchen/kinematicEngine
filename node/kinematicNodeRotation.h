/*
 * KinematicNodeRotation.h
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEROTATION_H_
#define KINEMATICNODEROTATION_H_

#include <utils/units.h>
#include "kinematicNode.h"

#include "../physics/ODEHingeMotor.h"

class KinematicNodeRotation: public KinematicNode {
public:
	KinematicNodeRotation();

	KinematicNodeRotation(
			MotorID id,
			KinematicNode *parent,
			std::string name,
			double minValue,
			double maxValue,
			double preferredValue,
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

	virtual void setValue(double newValue);

	virtual double getValue() const {
		return m_angle.value();
	}

	virtual bool isFixedNode() const
	{
		return false;
	}

	virtual arma::colvec3 getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const;

	virtual arma::colvec3 getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const;

	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const;

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame);

	virtual void setTorqueForODE(double torque);

protected:
	Degree m_angle;

	dJointID m_rotationJoint;

	double m_maxForce;
	RPM m_maxSpeed;

	ODEHingeMotor *m_hingeMotor;
};

#endif /* KINEMATICNODEROTATION_H_ */
