/*
 * kinematicNode.cpp
 *
 *  Created on: 12.02.2014
 *      Author: lutz
 */

#include "kinematicNode.h"
#include <limits.h>
#include "kinematicEngine/utils/homogeniousTransform.h"

namespace kinematicEngine {

KinematicNode::KinematicNode() :
					m_parent(NULL),
					m_name("uninitialized"),
					m_values(),                /* the actual rotation of the joint */
					m_valueDerivatives(),		/* the current speed of the joint */
					m_minValues(),             /* the minimum value (eg. rotation) of the joint */
					m_maxValues(),             /* the maximum value (eg. rotation) of the joint */
					m_preferredValues(),       /* preferred value (eg. rotation) of the joint */
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

KinematicNode::KinematicNode(KinematicNode *parent,
		std::string name,
		Millimeter translationX,
		Millimeter translationY,
		Millimeter translationZ,
		Degree alphaX,
		Degree alphaY,
		Degree alphaZ) :
	m_parent(parent),
	m_name(name),
	m_values(),                           /* the actual rotation of the joint */
	m_valueDerivatives(),                 /* the current speed of the joint */
	m_minValues(),                        /* the minimum value (eg. rotation) of the joint */
	m_maxValues(),                        /* the maximum value (eg. rotation) of the joint */
	m_preferredValues(),                  /* preferred value (eg. rotation) of the joint */
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
	preRotationMatrix = homogeneousTransform::getTransform(Meter(translationX), Meter(translationY), Meter(translationZ), m_alphaX, m_alphaY, m_alphaZ ) * m_additionalExtrinsicRotation;
	forwardMatrix = preRotationMatrix;
	// the handy way of inversion
	backwardMatrix = homogeneousTransform::invertHomogeneous(forwardMatrix);
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


void KinematicNode::setAdditionalExtrinsicRotation(arma::mat33 _rotMat) {
	preRotationMatrix = preRotationMatrix * arma::inv(m_additionalExtrinsicRotation);

	m_additionalExtrinsicRotation = arma::eye(4, 4);
	m_additionalExtrinsicRotation.submat(0, 0, 2, 2) = _rotMat.i();

	preRotationMatrix = preRotationMatrix * m_additionalExtrinsicRotation;
	setValues(m_values);
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

dMass KinematicNode::getODEMassToAttachToParent(arma::mat44 coordinateFrame) const {
	UNUSED(coordinateFrame);
	dMass dummyMass;
	dMassSetZero(&dummyMass);
	return dummyMass;
}

void KinematicNode::attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	for (KinematicVisual *visual : m_visuals)
	{
		visual->attatchToODE(coordinateFrame, body, space);
	}
}


arma::mat44 KinematicNode::getMatrixToRelative(KinematicNode const* relative) const
{
	arma::mat44 ret = arma::eye(4, 4);

	if ((NULL != m_parent) &&
		(relative == m_parent))
	{
		ret = backwardMatrix;
	} else
	{
		/* test the children for a match */
		for (const KinematicNode *child : m_children)
		{
			if (NULL != child)
			{
				if (relative == child)
				{
					/* found it */
					ret = child->forwardMatrix;
				}
			}
		}
	}
	return ret;
}

arma::mat44 KinematicNode::getInvMatrixToRelative(KinematicNode const* relative) const
{
	arma::mat44 ret = arma::eye(4, 4);

	if ((NULL != m_parent) &&
		(relative == m_parent))
	{
		ret = forwardMatrix;
	} else
	{
		/* test the children for a match */
		for (const KinematicNode *child : m_children)
		{
			if (NULL != child)
			{
				if (relative == child)
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

}
