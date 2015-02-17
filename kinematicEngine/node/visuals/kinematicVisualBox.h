/*
 * kinematicVisualBox.h
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#ifndef SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALBOX_H_
#define SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALBOX_H_

#include "kinematicVisual.h"

namespace kinematicEngine {

class KinematicVisualBox : public KinematicVisual {
public:
	KinematicVisualBox(
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
			bool isVisible = true,
			bool canCollide = true);

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

private:
	Millimeter sizeX, sizeY, sizeZ;
};

}

#endif /* SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALBOX_H_ */
