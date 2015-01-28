/*
 * physicsVisualization.h
 *
 *  Created on: 26.05.2014
 *      Author: lutz
 */

#ifndef PHYSICSVISUALIZATION_H_
#define PHYSICSVISUALIZATION_H_

#include "physicsEnvironment.h"
#include <utils/patterns/singleton.h>
#include <utils/system/thread.h>
#include <future>

class PhysicsVisualization : public Singleton<PhysicsVisualization> {

public:
	virtual ~PhysicsVisualization();

	void setEnvironmentToDraw(PhysicsEnvironment *env);

	void draw(int pause);

private:
	PhysicsVisualization();
	friend class Singleton<PhysicsVisualization>;

	/// the position we are looking at
	dReal lastObservedPosition[3];

	CriticalSection m_cs;
};

#endif /* PHYSICSVISUALIZATION_H_ */
