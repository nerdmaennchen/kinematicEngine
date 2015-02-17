/*
 * ODEUserObject.cpp
 *
 *  Created on: 02.06.2014
 *      Author: lutz
 */

#include "odeUserObject.h"

namespace kinematicEngine {

ODEUserObject::ODEUserObject()
	: canCollide(false)
	, textureNum(0)
	, visible(true)
{

}

ODEUserObject::~ODEUserObject() {
}

}

