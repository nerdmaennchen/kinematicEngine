#include "kinematicVisual.h"
#include "../physics/odeUtils.h"
#include "../physics/ODEUserObject.h"
#include "debug.h"

/*------------------------------------------------------------------------------------------------*/

/**
 **
 ** @param name
 ** @param translationX
 ** @param translationY
 ** @param translationZ
 ** @param sizeX
 ** @param sizeY
 ** @param sizeZ
 ** @param alphaX
 ** @param alphaY
 ** @param alphaZ
 ** @param isVisible
 */

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
			int textureNum,
			bool isVisible,
			bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, textureNum, isVisible, canCollide)
	, sizeX(sizeX)
	, sizeY(sizeY)
	, sizeZ(sizeZ)
{


	/*
	 *       4+--------+5
	 *       /.       /|             z   y
	 *      / .      / |             |  /
	 *    0+--------+1 |             | /
	 *     | 7+.....|..+6            |/
	 *     | .      | /              +------> x
	 *     |.       |/
	 *    3+--------+2
	 */

	arma::colvec4 vertices[8];
	vertices[0] = arma::colvec({ -Meter(sizeX).value()/2, -Meter(sizeY).value()/2, +Meter(sizeZ).value()/2, 1. });
	vertices[1] = arma::colvec({ +Meter(sizeX).value()/2, -Meter(sizeY).value()/2, +Meter(sizeZ).value()/2, 1. });
	vertices[2] = arma::colvec({ +Meter(sizeX).value()/2, -Meter(sizeY).value()/2, -Meter(sizeZ).value()/2, 1. });
	vertices[3] = arma::colvec({ -Meter(sizeX).value()/2, -Meter(sizeY).value()/2, -Meter(sizeZ).value()/2, 1. });
	vertices[4] = arma::colvec({ -Meter(sizeX).value()/2, +Meter(sizeY).value()/2, +Meter(sizeZ).value()/2, 1. });
	vertices[5] = arma::colvec({ +Meter(sizeX).value()/2, +Meter(sizeY).value()/2, +Meter(sizeZ).value()/2, 1. });
	vertices[6] = arma::colvec({ +Meter(sizeX).value()/2, +Meter(sizeY).value()/2, -Meter(sizeZ).value()/2, 1. });
	vertices[7] = arma::colvec({ -Meter(sizeX).value()/2, +Meter(sizeY).value()/2, -Meter(sizeZ).value()/2, 1. });

	for (int i=0; i<8; i++) {
		vertices[i] = transitionMatrix * vertices[i];
	}

	KinematicFace faceFront("Front");
	faceFront.addVertex(vertices[0]);
	faceFront.addVertex(vertices[3]);
	faceFront.addVertex(vertices[2]);
	faceFront.addVertex(vertices[1]);
	addFace(faceFront);

	KinematicFace faceRight("Right");
	faceRight.addVertex(vertices[1]);
	faceRight.addVertex(vertices[2]);
	faceRight.addVertex(vertices[6]);
	faceRight.addVertex(vertices[5]);
	addFace(faceRight);

	KinematicFace faceBack("Back");
	faceBack.addVertex(vertices[5]);
	faceBack.addVertex(vertices[6]);
	faceBack.addVertex(vertices[7]);
	faceBack.addVertex(vertices[4]);
	addFace(faceBack);

	KinematicFace faceLeft("Left");
	faceLeft.addVertex(vertices[0]);
	faceLeft.addVertex(vertices[4]);
	faceLeft.addVertex(vertices[7]);
	faceLeft.addVertex(vertices[3]);
	addFace(faceLeft);

	KinematicFace faceTop("Top");
	faceTop.addVertex(vertices[0]);
	faceTop.addVertex(vertices[1]);
	faceTop.addVertex(vertices[5]);
	faceTop.addVertex(vertices[4]);
	addFace(faceTop);

	KinematicFace faceBottom("Bottom");
	faceBottom.addVertex(vertices[7]);
	faceBottom.addVertex(vertices[6]);
	faceBottom.addVertex(vertices[2]);
	faceBottom.addVertex(vertices[3]);
	addFace(faceBottom);
}


void KinematicVisualBox::attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	dGeomID boxGeom = dCreateBox(space, Meter(sizeX).value()
									, Meter(sizeY).value()
									, Meter(sizeZ).value());
	ODEUserObject *userObj = new ODEUserObject;
	userObj->canCollide = canCollide;
	userObj->colorVec = m_colorVec;
	userObj->textureNum = m_textureNum;
	userObj->visible = visible;
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

/*------------------------------------------------------------------------------------------------*/

/**
 **
 ** @param transitionMatrix
 ** @param face
 ** @return
 */

KinematicFace operator* (arma::mat44 transitionMatrix, const KinematicFace& face) {
	KinematicFace newFace(face.getLabel());

	// transform the vectors into the camera coordinate system
	for (size_t i=0; i < face.size(); i++) {
		newFace.addVertex(transitionMatrix * face[i]);
	}

	return newFace;
}


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
		int textureNum,
		bool isVisible,
		bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, textureNum, isVisible, canCollide)
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
	userObj->colorVec = m_colorVec;
	userObj->textureNum = m_textureNum;
	userObj->visible = visible;
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
		int textureNum,
		bool isVisible,
		bool canCollide)
	: KinematicVisual(name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, colors, textureNum, isVisible, canCollide)
	, radius(radius)
{
}

void KinematicVisualSphere::attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space)
{
	dGeomID sphereGeom = dCreateSphere(space, Meter(radius).value());

	ODEUserObject *userObj = new ODEUserObject;
	userObj->canCollide = canCollide;
	userObj->colorVec = m_colorVec;
	userObj->textureNum = m_textureNum;
	userObj->visible = visible;

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
