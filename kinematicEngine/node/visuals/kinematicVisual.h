#ifndef KINEMATICVISUAL_H__
#define KINEMATICVISUAL_H__

#include "utils/units.h"

#include <armadillo>
#include <string>
#include <vector>
#include <array>

#include <ode/ode.h>
#include "kinematicEngine/utils/homogeniousTransform.h"

/*------------------------------------------------------------------------------------------------*/

namespace kinematicEngine {

class KinematicVisual {
public:
	typedef std::array<float, 4> ColorVec;

	KinematicVisual(std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			ColorVec colors,
			bool visible = true,
			bool canCollide = true)
		: m_colorVec(colors)
		, visible(visible)
		, canCollide(canCollide)
		, name(name)
	{
		transitionMatrix = homogeneousTransform::getTransform(Meter(translationX), Meter(translationY), Meter(translationZ), alphaX, alphaY, alphaZ);
	}

	virtual ~KinematicVisual() {}

	const std::string& getName() const { return name; };

	bool isVisible() const { return visible; }
	void setVisibility(bool isVisible) { visible = isVisible; }

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space) = 0;

protected:
	arma::mat44 transitionMatrix;

	ColorVec m_colorVec;
	bool visible;
	bool canCollide;

private:
	std::string name;
};

}


#endif
