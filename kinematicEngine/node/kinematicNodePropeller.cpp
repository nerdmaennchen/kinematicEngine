/*
 * KinematicNodeWheel.cpp
 *
 *  Created on: 24.10.2014
 *      Author: lutz
 */

#include "kinematicNodePropeller.h"
#include "kinematicEngine/physics/odeUtils.h"

namespace kinematicEngine {

KinematicNodePropeller::KinematicNodePropeller()
	: KinematicNodeWheel()
	, m_motor(nullptr)
	, m_speedToForceFactor(0.)
{
	// TODO Auto-generated constructor stub

}

KinematicNodePropeller::~KinematicNodePropeller() {
	// TODO Auto-generated destructor stub
}


KinematicNodePropeller::KinematicNodePropeller(KinematicNode *parent,
		std::string name,
		double maxForce,
		RPM maxSpeed,
		Millimeter translationX,
		Millimeter translationY,
		Millimeter translationZ,
		Degree alphaX,
		Degree alphaY,
		Degree alphaZ,
		double speedToForceFactor)
	: KinematicNodeWheel(parent, name, maxForce, maxSpeed, translationX, translationY, translationZ, alphaX, alphaY, alphaZ)
	, m_motor(nullptr)
	, m_speedToForceFactor(speedToForceFactor)
{

}

dBodyID KinematicNodePropeller::attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame)
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

	m_motor = new ODEPropellerMotor(m_motors[0], m_rotationJoint, environment, this, m_maxForce, m_maxSpeed, m_speedToForceFactor);

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

}
