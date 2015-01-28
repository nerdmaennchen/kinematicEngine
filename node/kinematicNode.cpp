/*
 * kinematicNode.cpp
 *
 *  Created on: 12.02.2014
 *      Author: lutz
 */

#include "kinematicNode.h"
#include <limits.h>

KinematicNode::KinematicNode() :
					id(MotorID(0)),
					m_parent(NULL),
					m_name("uninitialized"),
					m_value(0.),                /* the actual rotation of the joint */
					m_valueDerivative(0),		/* the current speed of the joint */
					m_minValue(0.),             /* the minimum value (eg. rotation) of the joint */
					m_maxValue(0.),             /* the maximum value (eg. rotation) of the joint */
					m_preferredValue(0.),       /* preferred value (eg. rotation) of the joint */
					m_translationX(0 * millimeters),      /* the translation along the x axis of the parent joint */
					m_translationY(0 * millimeters),      /* the translation along the x axis of the parent joint */
					m_translationZ(0 * millimeters),      /* the translation along the z axis of the parent joint */
					m_alphaX(0. * degrees),               /* the rotation around the x axis after the translation to match the joints z axis to the actual rotation axis */
					m_alphaY(0. * degrees),               /* same as above but around the y axis. this rotation is applied after the rotation around the x axis! */
					m_alphaZ(0. * degrees),               /* same as above but around the z axis. this rotation is applied after the rotation around the z axis! */
					m_hasMass(false),
					m_children(),
					forwardMatrix(arma::eye(4, 4)),
					backwardMatrix(arma::eye(4, 4)),
					preRotationMatrix(arma::eye(4, 4)),
					m_additionalExtrinsicRotation(arma::eye(4, 4))
{
}

KinematicNode::KinematicNode(MotorID id,
		KinematicNode *parent,
		std::string name,
		double minValue,
		double maxValue,
		double preferredValue,
		Millimeter translationX,
		Millimeter translationY,
		Millimeter translationZ,
		Degree alphaX,
		Degree alphaY,
		Degree alphaZ) :
	id(id),
	m_parent(parent),
	m_name(name),
	m_value(preferredValue),              /* the actual rotation of the joint */
	m_valueDerivative(0),                 /* the current speed of the joint */
	m_minValue(minValue),                 /* the minimum value (eg. rotation) of the joint */
	m_maxValue(maxValue),                 /* the maximum value (eg. rotation) of the joint */
	m_preferredValue(preferredValue),     /* preferred value (eg. rotation) of the joint */
	m_translationX(translationX),         /* the translation along the x axis of the parent joint */
	m_translationY(translationY),         /* the translation along the x axis of the parent joint */
	m_translationZ(translationZ),         /* the translation along the z axis of the parent joint */
	m_alphaX(alphaX),                     /* the rotation around the x axis after the translation to match the joints z axis to the actual rotation axis */
	m_alphaY(alphaY),                     /* same as above but around the y axis. this rotation is applied after the rotation around the x axis! */
	m_alphaZ(alphaZ),                     /* same as above but around the z axis. this rotation is applied after the rotation around the z axis! */
	m_hasMass(false),
	m_children(),
	m_additionalExtrinsicRotation(arma::eye(4, 4))
{
	arma::mat44 helperTrans = arma::eye(4, 4);
	helperTrans(0, 3) = Meter(m_translationX).value();
	helperTrans(1, 3) = Meter(m_translationY).value();
	helperTrans(2, 3) = Meter(m_translationZ).value();

	/* align z axis to current joint this might incorporate two rotations (first the rotation around x-axis is performed, then around the y axis and around the z axis) */
	arma::mat44 rotX = arma::eye(4, 4);
	arma::mat44 rotY = arma::eye(4, 4);
	arma::mat44 rotZ = arma::eye(4, 4);

	if (std::abs(m_alphaX.value()) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotX = cos(m_alphaX);
		const double sRotX = sin(m_alphaX);
		rotX(1, 1) = cRotX;
		rotX(1, 2) = -sRotX;
		rotX(2, 1) = sRotX;
		rotX(2, 2) = cRotX;
	}

	if (std::abs(m_alphaY.value()) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotY = cos(m_alphaY);
		const double sRotY = sin(m_alphaY);
		rotY(0, 0) = cRotY;
		rotY(0, 2) = sRotY;
		rotY(2, 0) = -sRotY;
		rotY(2, 2) = cRotY;
	}

	if (std::abs(m_alphaZ.value()) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotZ = cos(m_alphaZ);
		const double sRotZ = sin(m_alphaZ);
		rotZ(0, 0) = cRotZ;
		rotZ(0, 1) = -sRotZ;
		rotZ(1, 0) = sRotZ;
		rotZ(1, 1) = cRotZ;
	}

	preRotationMatrix = helperTrans * rotX * rotY * rotZ * m_additionalExtrinsicRotation;
	forwardMatrix = preRotationMatrix;
	// the handy way of inversion
	backwardMatrix = arma::inv(forwardMatrix);
}

KinematicNode::~KinematicNode() {
	for (KinematicVisual *visual : m_visuals)
	{
		delete visual;
	}

	if (0 != m_odeBody) {
		dBodyDestroy(m_odeBody);
	}
}


arma::mat44 KinematicNode::getMatrixToRelative(MotorID relative) const
{
	arma::mat44 ret = arma::eye(4, 4);

	if ((NULL != m_parent) &&
		(relative == m_parent->id))
	{
		ret = backwardMatrix;
	} else
	{
		/* test the children for a match */
		for (const KinematicNode *child : m_children)
		{
			if (NULL != child)
			{
				if (relative == child->id)
				{
					/* found it */
					ret = child->forwardMatrix;
				}
			}
		}
	}
	return ret;
}

arma::mat44 KinematicNode::getInvMatrixToRelative(MotorID relative) const
{
	arma::mat44 ret = arma::eye(4, 4);

	if ((NULL != m_parent) &&
		(relative == m_parent->id))
	{
		ret = forwardMatrix;
	} else
	{
		/* test the children for a match */
		for (const KinematicNode *child : m_children)
		{
			if (NULL != child)
			{
				if (relative == child->id)
				{
					/* found it */
					ret = child->backwardMatrix;
					break;
				}
			}
		}
	}
	return ret;
}

void KinematicNode::setAdditionalExtrinsicRotation(arma::mat33 _rotMat) {
	preRotationMatrix = preRotationMatrix * arma::inv(m_additionalExtrinsicRotation);

	m_additionalExtrinsicRotation = arma::eye(4, 4);
	m_additionalExtrinsicRotation.submat(0, 0, 2, 2) = _rotMat.i();

	preRotationMatrix = preRotationMatrix * m_additionalExtrinsicRotation;
	setValue(m_value);
}


dMass KinematicNode::getODEMass(arma::mat44 coordinateFrame)
{
	dMass compositeMass;
	dMassSetZero(&compositeMass);
	for (KinematicMass &mass : m_masses)
	{
		dMass componentMass = mass.getODEMass(coordinateFrame);
		if (componentMass.mass > 0.) {
			dMassAdd(&compositeMass, &componentMass);
		}
	}
	return compositeMass;
}


void KinematicNode::attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	for (KinematicVisual *visual : m_visuals)
	{
		visual->attatchToODE(coordinateFrame, body, space);
	}
}

