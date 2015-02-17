/*
 * kinematicVisualSphere.cpp
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#include "kinematicVisualSphere.h"
#include "kinematicEngine/physics/odeUtils.h"
#include "kinematicEngine/physics/odeUserObject.h"

namespace kinematicEngine {

KinematicVisualSphere::KinematicVisualSphere(
		std::string name,
		Millimeter translationX,
		Millimeter translationY,
		Millimeter translationZ,
		Millimeter radius,
		Degree alphaX,
		Degree alphaY,
		Degree alphaZ,
		KinematicVisual::ColorVec colors,
		bool isVisible,
		bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, isVisible, canCollide)
	, radius(radius)
{
}

void KinematicVisualSphere::attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	dGeomID sphereGeom = dCreateSphere(space, Meter(radius).value());

	ODEUserObject *userObj = new ODEUserObject;
	userObj->canCollide = canCollide;
	userObj->visible = visible;
	userObj->m_colorVec = m_colorVec;

	dGeomSetData(sphereGeom, userObj);
	dGeomSetBody(sphereGeom, body);

	arma::mat44 actualCoordinateFrame = coordinateFrame * transitionMatrix;

	dMatrix3 rotMat;
	dVector3 offsetVector;
	odeUtils::getRotationMatrixAsDMat(actualCoordinateFrame, rotMat);
	odeUtils::getPositionAsDVec(actualCoordinateFrame, offsetVector);

	dGeomSetOffsetPosition(sphereGeom, offsetVector[0]
									, offsetVector[1]
									, offsetVector[2]);
	dGeomSetOffsetRotation(sphereGeom, rotMat);
}

}
