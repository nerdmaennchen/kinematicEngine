/*
 * ODEUserObject.h
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#ifndef ODEUSEROBJECT_H_
#define ODEUSEROBJECT_H_

#include "../node/kinematicVisual.h"

class ODEUserObject {
public:
	ODEUserObject();
	virtual ~ODEUserObject();

	bool canCollide;

	KinematicVisual::ColorVec colorVec;
	int textureNum;
	bool visible;
};

#endif /* ODEUSEROBJECT_H_ */
