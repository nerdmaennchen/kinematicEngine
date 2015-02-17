/*
 * kinematicVisualSphere.h
 *
 *  Created on: 08.02.2015
 *      Author: lutz
 */

#ifndef SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALSPHERE_H_
#define SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALSPHERE_H_

#include "kinematicVisual.h"

namespace kinematicEngine {

class KinematicVisualSphere : public KinematicVisual {
public:
	KinematicVisualSphere(
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Millimeter radius,
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
#endif /* SRC_KINEMATICENGINE_NODE_VISUALS_KINEMATICVISUALSPHERE_H_ */
