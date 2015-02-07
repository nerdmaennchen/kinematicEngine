/*
 * physicsVisualization.h
 *
 *  Created on: 26.05.2014
 *      Author: lutz
 */

#ifndef PHYSICSVISUALIZATION_H_
#define PHYSICSVISUALIZATION_H_

#include "physicsEnvironment.h"
#include <mutex>

class PhysicsVisualization {

public:
	virtual ~PhysicsVisualization();

	void setEnvironmentToDraw(PhysicsEnvironment *env);

	void draw(int pause);

	static PhysicsVisualization& getInstance();

private:
	PhysicsVisualization();

	std::mutex m_mutex;
};

#endif /* PHYSICSVISUALIZATION_H_ */
