/*
 * physicsEnviroment.h
 *
 *  Created on: 23.05.2014
 *      Author: lutz
 */

#ifndef PHYSICSENVIROMENT_H_
#define PHYSICSENVIROMENT_H_

#include <map>

#include <ode/ode.h>
#include "utils/units.h"
#include "kinematicEngine/utils/motorIDs.h"

namespace kinematicEngine {

class PhysicsEnviromentPrivData;
class KinematicTree;
class ODEMotor;

//#define GRAVITY_EARTH (0)
#define GRAVITY_EARTH (-9.80665)
//#define GRAVITY_EARTH (-10)

class PhysicsEnvironmentStepCallback {
public:

	virtual ~PhysicsEnvironmentStepCallback() {};

	/**
	 * called from the simulation environment before each simulation step
	 */
	virtual void simulatorCallback(double timeDelta) = 0;
};

class PhysicsEnvironment {
public:
	PhysicsEnvironment();
	virtual ~PhysicsEnvironment();


	void setKinematicModel(KinematicTree *tree);

	void simulateStep(Second step);

	dSpaceID getCollisionSpaceID();
	dSpaceID getVisualsSpaceID();

	dWorldID getWorldID();

	void pauseSimulation();
	void unPauseSimulation();

	void addStepCallback(PhysicsEnvironmentStepCallback *callback);

	void addMotor(MotorID id, ODEMotor *motor);

	std::map<MotorID, ODEMotor*>& getMotors() {
		return m_motors;
	}

private:
	PhysicsEnviromentPrivData *m_privData;

	std::vector<PhysicsEnvironmentStepCallback*> m_stepCallbacks;

	std::map<MotorID, ODEMotor*> m_motors;
};

}

#endif /* PHYSICSENVIROMENT_H_ */
