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

#include "kinematicNodeRotation.h"
#include "kinematicEngine/physics/odeParallelMotor.h"

namespace kinematicEngine {

class KinematicNodeParallelRotation: public KinematicNodeRotation {
public:
	KinematicNodeParallelRotation();

	KinematicNodeParallelRotation(KinematicNode *parent,
			std::string name,
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

	virtual void setValues(MotorValuesMap newValues) override;

	virtual kinematics::JacobianValues getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const;

	virtual kinematics::JacobianValues getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const;

	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const;

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame);

	virtual void setTorqueForODE(double torque);

private:
	Millimeter m_limbLength;

	dBodyID m_intermediateBodyActive;
	dBodyID m_intermediateBodyPassive;
	dJointID m_activeRotationJointPre;   // here is the motor attached
	dJointID m_activeRotationJointPost;  // this is the extension of the active joint to the "body to be moved"
	dJointID m_passiveRotationJointPre;  // this is the other joint helping everything to stay in place
	dJointID m_passiveRotationJointPost; // this is the other joint helping everything to stay in place (2nd part)

	ODEParallelMotor *m_odeMotor;
};

}

#endif /* KINEMATICNODEPARALLELROTATION_H_ */
