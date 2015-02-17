/*
 * kinematicVisualBox.cpp
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#include "kinematicVisualBox.h"
#include "kinematicEngine/physics/odeUtils.h"
#include "kinematicEngine/physics/odeUserObject.h"

namespace kinematicEngine {

KinematicVisualBox::KinematicVisualBox(
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Millimeter sizeX,
			Millimeter sizeY,
			Millimeter sizeZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			KinematicVisual::ColorVec colors,
			bool isVisible,
			bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, isVisible, canCollide)
	, sizeX(sizeX)
	, sizeY(sizeY)
	, sizeZ(sizeZ)
{
}


void KinematicVisualBox::attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	dGeomID boxGeom = dCreateBox(space, Meter(sizeX).value()
									, Meter(sizeY).value()
									, Meter(sizeZ).value());
	ODEUserObject *userObj = new ODEUserObject;
	userObj->canCollide = canCollide;
	userObj->visible = visible;
	userObj->m_colorVec = m_colorVec;
	dGeomSetData(boxGeom, userObj);

	dGeomSetBody(boxGeom, body);

	arma::mat44 actualCoordinateFrame = coordinateFrame * transitionMatrix;

	dMatrix3 rotMat;
	dVector3 offsetVector;
	odeUtils::getRotationMatrixAsDMat(actualCoordinateFrame, rotMat);
	odeUtils::getPositionAsDVec(actualCoordinateFrame, offsetVector);

	dGeomSetOffsetPosition(boxGeom, offsetVector[0]
									, offsetVector[1]
									, offsetVector[2]);
	dGeomSetOffsetRotation(boxGeom, rotMat);
}

}
