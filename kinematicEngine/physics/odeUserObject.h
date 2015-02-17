/*
 * ODEUserObject.h
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#ifndef ODEUSEROBJECT_H_
#define ODEUSEROBJECT_H_

#include "kinematicEngine/node/visuals/kinematicVisual.h"

namespace kinematicEngine {

class ODEUserObject {
public:
	typedef std::array<float, 4> ColorVec;

	ODEUserObject();
	virtual ~ODEUserObject();

	bool canCollide;

	int textureNum;
	bool visible;
	ColorVec m_colorVec;
};

}

#endif /* ODEUSEROBJECT_H_ */
