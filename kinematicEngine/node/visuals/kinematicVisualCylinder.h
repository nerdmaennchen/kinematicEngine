/*
 * kinematicVisualCylinder.h
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#ifndef SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALCYLINDER_H_
#define SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALCYLINDER_H_

#include "kinematicVisual.h"

namespace kinematicEngine {

class KinematicVisualCylinder : public KinematicVisual {
public:
	KinematicVisualCylinder(
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
			bool isVisible = true,
			bool canCollide = true);

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

private:
	Millimeter radius, length;
};

}

#endif /* SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALCYLINDER_H_ */
