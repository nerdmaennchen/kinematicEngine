/*
 * kinematicNodeDummy.cpp
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#include "kinematicNodeDummy.h"

#include "kinematicEngine/physics/odeUtils.h"

namespace kinematicEngine {

KinematicNodeDummy::KinematicNodeDummy() : KinematicNode() {
}

KinematicNodeDummy::KinematicNodeDummy(KinematicNode *parent,
										std::string name,
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
{
}

KinematicNodeDummy::~KinematicNodeDummy() {
}


kinematics::JacobianValues KinematicNodeDummy::getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const
{
	UNUSED(effector);
	kinematics::JacobianValues ret;
	return ret;
}

kinematics::JacobianValues KinematicNodeDummy::getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const
{
	UNUSED(effector);
	kinematics::JacobianValues ret;
	return ret;
}

arma::colvec4 KinematicNodeDummy::getLinearSpeedOfPoint(arma::colvec3 position) const
{
	UNUSED(position);
	return arma::zeros(4);
}

dBodyID KinematicNodeDummy::attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 parentsCoordinateFrame)
{
	// move coordinate frame to current node:
	arma::mat44 coordinateFrame = parentsCoordinateFrame * forwardMatrix;
	dWorldID worldID = environment->getWorldID();

	if (nullptr == m_parent)
	{
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

	} else {
		// setup the nodeBodyOffset based on the parent's odeBody

		dBodyID body = this->getODEBody();
		const dReal *bodyPosition = dBodyGetPosition(body);
		const dReal *bodyRotation = dBodyGetRotation(body);
		arma::mat44 bodyFrame = odeUtils::getMatFromDMatAndDVec(bodyRotation, bodyPosition);
		arma::mat44 helperRotation = arma::eye(4, 4);

		helperRotation.submat(0, 0, 2, 2) = coordinateFrame.submat(0, 0, 2, 2);

		m_odeNodeBodyOffset = coordinateFrame.i() * bodyFrame;
	}

	// create a "box" to visualize the node
	dGeomID geom = dCreateSphere(visualSpaceID, 0.005);
	dMatrix3 bodyRotation;
	dVector3 bodyPosition;

	// the visuals are defined as "local" coordinates but here we need to incorporate the body offset
	arma::mat44 visualsFrame = m_odeNodeBodyOffset.i();
	odeUtils::getRotationMatrixAsDMat(visualsFrame, bodyRotation); // inverse of rotation part
	odeUtils::getPositionAsDVec(visualsFrame, bodyPosition);

	dGeomSetBody(geom, getODEBody());
	dGeomSetOffsetRotation(geom, bodyRotation);
	dGeomSetOffsetPosition(geom, bodyPosition[0], bodyPosition[1], bodyPosition[2]);

	// finally add all the visuals
	this->attatchODEVisuals(visualsFrame, getODEBody(), collisionSpaceID);


	// recurse into everything attached to this node
	for (KinematicNode *child : m_children)
	{
		dBodyID childBody = child->attachToODE(environment, visualSpaceID, collisionSpaceID, coordinateFrame);
		UNUSED(childBody);
	}

	return m_odeBody;
}

dMass KinematicNodeDummy::getODEMassToAttachToParent(arma::mat44 coordinateFrame) const
{
	dMass compositeMass;
	dMassSetZero(&compositeMass);
	for (KinematicMass const& mass : m_masses)
	{
		dMass componentMass = mass.getODEMass(coordinateFrame);
		dMassAdd(&compositeMass, &componentMass);
	}

	for (KinematicNode *child : m_children)
	{
		arma::mat44 childFrame = coordinateFrame * child->getForwardMatrix();
		dMass componentMass = child->getODEMassToAttachToParent(childFrame);
		dMassAdd(&compositeMass, &componentMass);
	}

	return compositeMass;
}

dBodyID KinematicNodeDummy::getODEBody() const
{
	if (nullptr != m_parent)
	{
		return m_parent->getODEBody();
	} else {
		return m_odeBody;
	}
}

void KinematicNodeDummy::attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	for (KinematicVisual *visual : m_visuals)
	{
		visual->attatchToODE(coordinateFrame, body, space);
	}
}

void KinematicNodeDummy::setTorqueForODE(double torque)
{
	// nothing to do
	UNUSED(torque);
}

}
