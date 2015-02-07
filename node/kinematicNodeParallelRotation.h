/*
 * kinematicNodeParallelRotation.h
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEPARALLELROTATION_H_
#define KINEMATICNODEPARALLELROTATION_H_

#include "utils/units.h"
#include "kinematicNode.h"

#include "../physics/ODEParallelMotor.h"

class KinematicNodeParallelRotation: public KinematicNode {
public:
	KinematicNodeParallelRotation();

	KinematicNodeParallelRotation(MotorID id,
			KinematicNode *parent,
			std::string name,
			Degree minValue,
			Degree maxValue,
			Degree preferredValue,
			double maxForce,
			RPM maxSpeed,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			Millimeter limbLength);

	virtual ~KinematicNodeParallelRotation();

	/// create a copy of this node without parent and children
	virtual KinematicNode* cloneDetached() const override {
		KinematicNodeParallelRotation* node = new KinematicNodeParallelRotation();
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

private:
	Radian m_angle;
	Millimeter m_limbLength;

	dBodyID m_intermediateBodyActive;
	dBodyID m_intermediateBodyPassive;
	dJointID m_activeRotationJointPre;   // here is the motor attached
	dJointID m_activeRotationJointPost;  // this is the extension of the active joint to the "body to be moved"
	dJointID m_passiveRotationJointPre;  // this is the other joint helping everything to stay in place
	dJointID m_passiveRotationJointPost; // this is the other joint helping everything to stay in place (2nd part)

	ODEParallelMotor *m_odeMotor;

	double m_maxForce;
	double m_maxSpeed;
};

#endif /* KINEMATICNODEPARALLELROTATION_H_ */
