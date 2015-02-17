/*
 * kinematicNodeParallelRotation.cpp
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#include "kinematicNodeParallelRotation.h"
#include "kinematicEngine/physics/odeUtils.h"


namespace kinematicEngine {

KinematicNodeParallelRotation::KinematicNodeParallelRotation()
		: KinematicNodeRotation()
		, m_limbLength(0 * millimeters)
{
	setValues(m_preferredValues);
}


KinematicNodeParallelRotation::KinematicNodeParallelRotation(KinematicNode *parent,
											std::string name,
											double maxForce,
											RPM maxSpeed,
											Millimeter translationX,
											Millimeter translationY,
											Millimeter translationZ,
											Degree alphaX,
											Degree alphaY,
											Degree alphaZ,
											Millimeter limbLength)
		: KinematicNodeRotation(parent,
						name,
						maxForce,
						maxSpeed,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ)
		, m_limbLength(limbLength)
{
	setValues(m_preferredValues);
}

KinematicNodeParallelRotation::~KinematicNodeParallelRotation() {
	delete m_odeMotor;
	if (0 != m_activeRotationJointPost)
		dJointDestroy(m_activeRotationJointPost);
	if (0 != m_activeRotationJointPre)
		dJointDestroy(m_activeRotationJointPre);
	if (0 != m_passiveRotationJointPost)
		dJointDestroy(m_passiveRotationJointPost);
	if (0 != m_passiveRotationJointPre)
		dJointDestroy(m_passiveRotationJointPre);
	if (0 != m_intermediateBodyActive)
		dBodyDestroy(m_intermediateBodyActive);
	if (0 != m_intermediateBodyPassive)
		dBodyDestroy(m_intermediateBodyPassive);
}


void KinematicNodeParallelRotation::setValues(MotorValuesMap newValues)
{
	KinematicNode::setValues(newValues);

	const double angle = newValues.begin()->second;
	arma::mat44 rotation = arma::eye(4, 4);

	const double cRot = cos(angle);
	const double sRot = sin(angle);

	/* we are a joint doing parallel kinematics */
	rotation(0, 3) = cRot * Meter(m_limbLength).value();
	rotation(1, 3) = sRot * Meter(m_limbLength).value();

	forwardMatrix = preRotationMatrix * rotation;
	// the handy way of inversion
	backwardMatrix = arma::inv(forwardMatrix);
}


kinematics::JacobianValues KinematicNodeParallelRotation::getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const
{
	UNUSED(effector); // the position of the effector doesnt really matter...
	kinematics::JacobianValues ret;

	const double angle = m_values.begin()->second;

	const double cRot = cos(angle);
	const double sRot = sin(angle);
	/* in this case the partial derivative is just the length of this joint cross multiplied with the rotation axis vector */

	const arma::colvec3 crank = arma::colvec({cRot, sRot, 0}) * Meter(m_limbLength).value();
	const arma::colvec3 rotationAxis = arma::colvec({0, 0, 1});

	ret.push_back({m_motor2IntMap.begin()->second, arma::cross(rotationAxis, crank)});
	return ret;
}

kinematics::JacobianValues KinematicNodeParallelRotation::getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const
{
	kinematics::JacobianValues ret;

	UNUSED(effector); // cannot rotate
	ret.push_back({m_motor2IntMap.begin()->second, arma::zeros(3)});

	return ret;
}


arma::colvec4 KinematicNodeParallelRotation::getLinearSpeedOfPoint(arma::colvec3 position) const
{
	return arma::colvec({-position(1), position(0), 0, 0}) * m_valueDerivatives.begin()->first;
}

dBodyID KinematicNodeParallelRotation::attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame)
{
	// move coordinate frame to current node:
	const Centimeter offset = 5. * centimeters;
	arma::mat44 offsetMat = arma::eye(4, 4);
	offsetMat(1, 3) = Meter(offset).value();

	arma::mat44 coordinateFrame     = parentsCoordinateFrame * preRotationMatrix;
	arma::mat44 coordinateFramePost = parentsCoordinateFrame * forwardMatrix;

	arma::mat44 coordinateFrameOffset     = parentsCoordinateFrame * preRotationMatrix * offsetMat;
	arma::mat44 coordinateFramePostOffset = parentsCoordinateFrame * forwardMatrix * offsetMat;


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

	// create two bodies to attach the active and passive joints to
	m_intermediateBodyActive  = dBodyCreate(worldID);
	m_intermediateBodyPassive = dBodyCreate(worldID);

	dMass dummyMass;
	dMassSetSphereTotal(&dummyMass, 1. / 1000., 0.1);

	dBodySetMass(m_intermediateBodyActive, &dummyMass);
	dBodySetMass(m_intermediateBodyPassive, &dummyMass);

	arma::mat44 globalSystemBodyOffset = coordinateFramePost * m_odeNodeBodyOffset;

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

	dVector3 intermediateBodyPosition;
	odeUtils::getPositionAsDVec((coordinateFrame + coordinateFramePost) * 0.5, intermediateBodyPosition);
	dBodySetPosition(m_intermediateBodyActive, intermediateBodyPosition[0], intermediateBodyPosition[1], intermediateBodyPosition[2]);
	dBodySetRotation(m_intermediateBodyActive, bodyRotation);

	odeUtils::getPositionAsDVec((coordinateFrameOffset + coordinateFramePostOffset) * 0.5, intermediateBodyPosition);
	dBodySetPosition(m_intermediateBodyPassive, intermediateBodyPosition[0], intermediateBodyPosition[1], intermediateBodyPosition[2]);
	dBodySetRotation(m_intermediateBodyPassive, bodyRotation);

	// create a "box" to visualize the node and to have collision detection support
	dGeomID geom = dCreateBox(visualSpaceID, 0.002, 0.001, 0.001);
	dGeomSetBody(geom, m_odeBody);
	dGeomSetOffsetPosition(geom, -m_odeNodeBodyOffset(0, 3), -m_odeNodeBodyOffset(1, 3), -m_odeNodeBodyOffset(2, 3));

	m_activeRotationJointPre   = dJointCreateHinge(worldID, 0);
	m_passiveRotationJointPre  = dJointCreateHinge(worldID, 0);
	m_activeRotationJointPost  = dJointCreateHinge(worldID, 0);
	m_passiveRotationJointPost = dJointCreateHinge(worldID, 0);

	dBodyID parentBody = 0;
	if (m_parent) {
		parentBody = m_parent->getODEBody();
	}

	dJointAttach(m_activeRotationJointPre,  m_intermediateBodyActive, parentBody);
	dJointAttach(m_passiveRotationJointPre, m_intermediateBodyPassive, parentBody);

	dJointAttach(m_activeRotationJointPost,  m_odeBody, m_intermediateBodyActive);
	dJointAttach(m_passiveRotationJointPost, m_odeBody, m_intermediateBodyPassive);

	dJointSetHingeAnchor(m_activeRotationJointPre,  coordinateFrame(0, 3), coordinateFrame(1, 3), coordinateFrame(2, 3));
	dJointSetHingeAnchor(m_activeRotationJointPost, coordinateFramePost(0, 3), coordinateFramePost(1, 3), coordinateFramePost(2, 3));

	dJointSetHingeAxis(m_activeRotationJointPre, coordinateFrame(0, 2), coordinateFrame(1, 2), coordinateFrame(2, 2));
	dJointSetHingeAxis(m_activeRotationJointPost, coordinateFramePost(0, 2), coordinateFramePost(1, 2), coordinateFramePost(2, 2));

	dJointSetHingeAnchor(m_passiveRotationJointPre,  coordinateFrameOffset(0, 3), coordinateFrameOffset(1, 3), coordinateFrameOffset(2, 3));
	dJointSetHingeAnchor(m_passiveRotationJointPost, coordinateFramePostOffset(0, 3), coordinateFramePostOffset(1, 3), coordinateFramePostOffset(2, 3));

	dJointSetHingeAxis(m_passiveRotationJointPre, coordinateFrameOffset(0, 2), coordinateFrameOffset(1, 2), coordinateFrameOffset(2, 2));
	dJointSetHingeAxis(m_passiveRotationJointPost, coordinateFramePostOffset(0, 2), coordinateFramePostOffset(1, 2), coordinateFramePostOffset(2, 2));

	m_odeMotor = new ODEParallelMotor(m_motors[0], m_activeRotationJointPre, environment, this, m_maxForce, m_preferredValues.begin()->second, m_minValues.begin()->second, m_maxValues.begin()->second, m_maxSpeed);
	m_odeMotor->setValueOffset(m_values.begin()->second);

	// finally add all the visuals
	// the visuals are defined as "local" coordinates but here we need to incorporate the body offset
	arma::mat44 visualsFrame = m_odeNodeBodyOffset.i();
	this->attatchODEVisuals(visualsFrame, getODEBody(), collisionSpaceID);

	// recurse into everything attached to this node
	for (KinematicNode *child : m_children)
	{
		dBodyID childBody = child->attachToODE(environment, visualSpaceID, collisionSpaceID, coordinateFramePost);
		UNUSED(childBody);
	}

	return m_odeBody;
}

void KinematicNodeParallelRotation::setTorqueForODE(double torque)
{
	dJointAddHingeTorque(m_activeRotationJointPre, torque);
}

}
