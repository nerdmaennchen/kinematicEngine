/*
 * kinematicVisualCylinder.cpp
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#include "kinematicVisualCylinder.h"
#include "kinematicEngine/physics/odeUtils.h"
#include "kinematicEngine/physics/odeUserObject.h"

namespace kinematicEngine {

KinematicVisualCylinder::KinematicVisualCylinder(
		std::string name,
		Millimeter translationX,
		Millimeter translationY,
		Millimeter translationZ,
		Millimeter radius,
		Millimeter length,
		Degree alphaX,
		Degree alphaY,
		Degree alphaZ,
		KinematicVisual::ColorVec colors,
		bool isVisible,
		bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, isVisible, canCollide)
	, radius(radius)
	, length(length)
{
}

void KinematicVisualCylinder::attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	dGeomID cylingerGeom = dCreateCylinder(space, Meter(radius).value()
									, Meter(length).value());

	ODEUserObject *userObj = new ODEUserObject;
	userObj->canCollide = canCollide;
	userObj->visible = visible;
	userObj->m_colorVec = m_colorVec;
	dGeomSetData(cylingerGeom, userObj);

	dGeomSetBody(cylingerGeom, body);

	arma::mat44 actualCoordinateFrame = coordinateFrame * transitionMatrix;

	dMatrix3 rotMat;
	dVector3 offsetVector;
	odeUtils::getRotationMatrixAsDMat(actualCoordinateFrame, rotMat);
	odeUtils::getPositionAsDVec(actualCoordinateFrame, offsetVector);

	dGeomSetOffsetPosition(cylingerGeom, offsetVector[0]
									, offsetVector[1]
									, offsetVector[2]);
	dGeomSetOffsetRotation(cylingerGeom, rotMat);
}

}
