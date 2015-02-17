/*
 * KinematicNodeRotation.cpp
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#include "kinematicNodeRotation.h"
#include "kinematicEngine/physics/odeUtils.h"

namespace kinematicEngine {

KinematicNodeRotation::KinematicNodeRotation() : KinematicNode() {
	setValues(m_preferredValues);
}


KinematicNodeRotation::KinematicNodeRotation(KinematicNode *parent,
											std::string name,
											double maxForce,
											RPM maxSpeed,
											Millimeter translationX,
											Millimeter translationY,
											Millimeter translationZ,
											Degree alphaX,
											Degree alphaY,
											Degree alphaZ)
		: KinematicNode(parent,
						name,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ)
		, m_maxForce(maxForce)
		, m_maxSpeed(maxSpeed.value() * 2. * M_PI / 60.)
{
	setValues(m_preferredValues);
}

KinematicNodeRotation::~KinematicNodeRotation() {
	if (nullptr != m_hingeMotor) {
		delete m_hingeMotor;
	}
	if (0 != m_rotationJoint) {
		dJointDestroy(m_rotationJoint);
	}
}



void KinematicNodeRotation::setValues(MotorValuesMap newValues)
{
	KinematicNode::setValues(newValues);

	const double angle = newValues.begin()->second;

	arma::mat44 rotation = arma::eye(4, 4);
	const double cRot = cos(angle);
	const double sRot = sin(angle);

	rotation(0, 0) = cRot;
	rotation(0, 1) = -sRot;
	rotation(1, 0) = sRot;
	rotation(1, 1) = cRot;

	forwardMatrix = preRotationMatrix * rotation;
	// the handy way of inversion
	backwardMatrix = arma::inv(forwardMatrix);
}


kinematics::JacobianValues KinematicNodeRotation::getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const
{
	kinematics::JacobianValues ret;

	arma::colvec3 vec = arma::zeros(3);
	/* in this case the partial derivative of the effector's movement is the cross product of the difference vector of this' joint's position and it's rotation axis */
	vec(0) = -effector(1);
	vec(1) = effector(0);

	ret.push_back({m_motor2IntMap.begin()->second, vec});
	return ret;
}

kinematics::JacobianValues KinematicNodeRotation::getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const
{
	kinematics::JacobianValues ret;

	arma::colvec3 vec = arma::zeros(3);
	/* in this case the partial derivative of the effector's movement is the cross product of the difference vector of this' joint's position and it's rotation axis */
	vec(0) = -effector(1);
	vec(1) = effector(0);

	ret.push_back({m_motor2IntMap.begin()->second, vec});
	return ret;
}


arma::colvec4 KinematicNodeRotation::getLinearSpeedOfPoint(arma::colvec3 position) const
{
	return arma::colvec({-position(1), position(0), 0, 0}) * m_valueDerivatives.begin()->second;
}

dBodyID KinematicNodeRotation::attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame)
{
	// move coordinate frame to current node:
	arma::mat44 coordinateFrame = parentsCoordinateFrame * forwardMatrix;
	dWorldID worldID = environment->getWorldID();

	// the body must coincide with the center of mass of the equivalent mass of all dummy children and this nodes masses
	arma::mat44 frame = arma::eye(4, 4);
	dMass equivMass = this->getODEMass(frame);
	for (KinematicNode *child : m_children)
	{
		arma::mat44 childFrame = frame * child->getForwardMatrix();
		dMass childMass = child->getODEMassToAttachToParent(childFrame);
		if (childMass.mass > 0.) {
			dMassAdd(&equivMass, &childMass);
		}
	}

	m_odeNodeBodyOffset = arma::eye(4, 4);
	m_odeNodeBodyOffset(0, 3) = equivMass.c[0];
	m_odeNodeBodyOffset(1, 3) = equivMass.c[1];
	m_odeNodeBodyOffset(2, 3) = equivMass.c[2];

	dMassTranslate(&equivMass, -m_odeNodeBodyOffset(0, 3), -m_odeNodeBodyOffset(1, 3), -m_odeNodeBodyOffset(2, 3));

	// create a body
	m_odeBody = dBodyCreate(worldID);

	arma::mat44 globalSystemBodyOffset = coordinateFrame * m_odeNodeBodyOffset;

	dMatrix3 bodyRotation;
	dVector3 bodyPosition;

	odeUtils::getRotationMatrixAsDMat(globalSystemBodyOffset, bodyRotation);
	odeUtils::getPositionAsDVec(globalSystemBodyOffset, bodyPosition);


	dBodySetPosition(m_odeBody, bodyPosition[0], bodyPosition[1], bodyPosition[2]);
	dBodySetRotation(m_odeBody, bodyRotation);
	if (equivMass.mass > 0.)
	{
		dBodySetMass(m_odeBody, &equivMass);
	} else {
		dBodyGetMass(m_odeBody, &equivMass);
		equivMass.mass = std::numeric_limits<double>::min();
		dBodySetMass(m_odeBody, &equivMass);
	}


	dBodyID parentBody = 0;
	if (m_parent) {
		parentBody = m_parent->getODEBody();
	}

	m_rotationJoint = dJointCreateHinge(worldID, 0);
	dJointAttach(m_rotationJoint, m_odeBody, parentBody);

	dJointSetHingeAnchor(m_rotationJoint, coordinateFrame(0, 3), coordinateFrame(1, 3), coordinateFrame(2, 3));
	dJointSetHingeAxis(m_rotationJoint, coordinateFrame(0, 2), coordinateFrame(1, 2), coordinateFrame(2, 2));

	m_hingeMotor = new ODEHingeMotor(m_motors[0],
			m_rotationJoint,
			environment,
			this,
			m_maxForce,
			m_preferredValues.begin()->second,
			m_minValues.begin()->second,
			m_maxValues.begin()->second,
			m_maxSpeed);

	// create a "box" to visualize the node and to have collision detection support
	dGeomID geom = dCreateBox(visualSpaceID, 0.02, 0.01, 0.01);
	dGeomSetBody(geom, m_odeBody);
	dGeomSetOffsetPosition(geom, -m_odeNodeBodyOffset(0, 3), -m_odeNodeBodyOffset(1, 3), -m_odeNodeBodyOffset(2, 3));

	// finally add all the visuals
	// the visuals are defined as "local" coordinates but here we need to incorporate the body offset
	arma::mat44 visualsFrame = m_odeNodeBodyOffset.i();
	this->attatchODEVisuals(visualsFrame, m_odeBody, collisionSpaceID);

	// recurse into everything attached to this node
	for (KinematicNode *child : m_children)
	{
		dBodyID childBody = child->attachToODE(environment, visualSpaceID, collisionSpaceID, coordinateFrame);
		UNUSED(childBody);
	}

	return m_odeBody;
}


void KinematicNodeRotation::setTorqueForODE(double torque)
{
	dJointAddHingeTorque(m_rotationJoint, torque);
}

}
