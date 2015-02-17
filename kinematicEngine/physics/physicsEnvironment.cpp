/*
 * physicsEnviroment.cpp
 *
 *  Created on: 23.05.2014
 *      Author: lutz
 */

#include "physicsEnvironment.h"
#include <mutex>
#include "odeUserObject.h"
#include "kinematicEngine/kinematicTree.h"
#include "odeMotor.h"


namespace kinematicEngine {

static void nearCallback_wrapper (void *data, dGeomID o1, dGeomID o2);

class PhysicsEnviromentPrivData
{

public:
	dWorld m_world;
	dHashSpace m_collisionSpace;
	dHashSpace m_visualSpace;
	dJointGroup m_contactJoints;
	dPlane m_groundPlane;
	std::mutex m_mutex;

	PhysicsEnviromentPrivData() : m_world(), m_collisionSpace(), m_visualSpace(), m_groundPlane()
	{
		m_groundPlane.create(m_collisionSpace, 0., 0., 1., 0.);

		ODEUserObject *planeUserData = new ODEUserObject;
		planeUserData->canCollide = true; // this is the ground...
		m_groundPlane.setData(planeUserData);

		m_world.setGravity(0., 0., GRAVITY_EARTH);
	}

	~PhysicsEnviromentPrivData()
	{}

	void setKinematicModel(PhysicsEnvironment *environment, arma::mat44 coordinateFrame, KinematicNode *rootNode)
	{
		rootNode->attachToODE(environment, m_visualSpace.id(), m_collisionSpace.id(), coordinateFrame);
	}

	void performStep(double timeDist)
	{
		m_collisionSpace.collide((void *)this, &nearCallback_wrapper);
		m_world.step(timeDist);
		m_contactJoints.clear();
	}

	static void initODE_once()
	{
		static bool isInitialized = false;
		if (false == isInitialized)
		{
			dInitODE();
			isInitialized = true;
		}
	}

	void nearCallback (dGeomID o1, dGeomID o2)
	{
		if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
		{
			// colliding a space with something
			dSpaceCollide2(o1, o2, this, &nearCallback_wrapper);
			// Note we do not want to test intersections within a space,
			// only between spaces.
			return;
		}

		if (dGeomGetBody(o1) == dGeomGetBody(o2))
		{
			return; // no collision when the geoms belong to the same body
		}

		ODEUserObject *uo1 = (ODEUserObject*) dGeomGetData(o1);
		ODEUserObject *uo2 = (ODEUserObject*) dGeomGetData(o2);
		if (nullptr != uo1 && nullptr != uo2)
		{
			if (uo1->canCollide && uo2->canCollide)
			{
				const int numGeoms = m_collisionSpace.getNumGeoms();
				dContact* contact = new dContact[numGeoms];
				int n = dCollide(o1, o2, numGeoms, &(contact[0].geom), sizeof(dContact));
				if (n > 0)
				{
					for (int i=0; i<n; i++)
					{
						contact[i].surface.mode = dContactSoftCFM | dContactApprox1;
						contact[i].surface.mu = 2;
						contact[i].surface.soft_erp = 0.96;
						contact[i].surface.soft_cfm = 0.001;
						dJointID c = dJointCreateContact (m_world, m_contactJoints.id(), &contact[i]);
						dJointAttach (c,
							dGeomGetBody(contact[i].geom.g1),
							dGeomGetBody(contact[i].geom.g2));
					}
				}
				delete contact;
			}
		}
	}

	void offsetAllBodies(KinematicTree *tree) {
		std::vector<dBodyID> allBodies;
		getAllBodies(tree->getRootNode(), allBodies);

		dReal aabb[6];
		for (int i = 0; i < 6; ++i) {
			aabb[i] = 0;
		}

		for (dBodyID body : allBodies) {
			dGeomID geom = dBodyGetFirstGeom(body);
			while (geom != nullptr)
			{
				dReal curAABB[6];
				dGeomGetAABB(geom, curAABB);

				for (uint i = 0; i < 6; i += 2) {
					aabb[i + 0] = std::min(aabb[i + 0], curAABB[i + 0]);
					aabb[i + 1] = std::max(aabb[i + 1], curAABB[i + 1]);
				}

				geom = dBodyGetNextGeom(geom);
			}
		}

		dReal XOff = 0;
		dReal YOff = 0;
		dReal ZOff = -aabb[4];

		for (dBodyID body : allBodies) {
		    const dReal *pos = dBodyGetPosition(body);
		    dBodySetPosition(body, pos[0] + XOff, pos[1] + YOff, pos[2] + ZOff);
		}

	}
	void getAllBodies(KinematicNode const* node, std::vector<dBodyID> &o_bodies) {
		dBodyID nodesBody = node->getODEBody();
		if (std::find(o_bodies.begin(), o_bodies.end(), nodesBody) == o_bodies.end()) {
			o_bodies.push_back(nodesBody);
		}

		for (KinematicNode const* child : node->getChildren()) {
			getAllBodies(child, o_bodies);
		}
	}
};


void nearCallback_wrapper (void *data, dGeomID o1, dGeomID o2)
{
	if (nullptr != data)
	{
		((PhysicsEnviromentPrivData*)data)->nearCallback(o1, o2);
	}
}



PhysicsEnvironment::PhysicsEnvironment() {
	PhysicsEnviromentPrivData::initODE_once();
	m_privData = new PhysicsEnviromentPrivData();
}

PhysicsEnvironment::~PhysicsEnvironment() {
}



void PhysicsEnvironment::setKinematicModel(KinematicTree *tree)
{	// get all the stuff from the tree and put it into the physics environment

	arma::mat44 coordinateFrame = arma::eye(4, 4);
	std::map<kinematics::NodeID, arma::colvec3> effectorPositions;
	KinematicNode *rootNode = const_cast<KinematicNode*>(tree->getRootNode());
	tree->calculateEffectorsPositions(effectorPositions, rootNode, coordinateFrame);

	double minX =  std::numeric_limits<double>::infinity();
	double maxX = -std::numeric_limits<double>::infinity();
	double minY =  std::numeric_limits<double>::infinity();
	double maxY = -std::numeric_limits<double>::infinity();
	double minZ =  std::numeric_limits<double>::infinity();
	double maxZ = -std::numeric_limits<double>::infinity();

	for (auto pos : effectorPositions)
	{
		minX = std::min(minX, pos.second(0));
		minY = std::min(minY, pos.second(1));
		minZ = std::min(minZ, pos.second(2));

		maxX = std::max(maxX, pos.second(0));
		maxY = std::max(maxY, pos.second(1));
		maxZ = std::max(maxZ, pos.second(2));
	}


	arma::mat44 rootFrame = arma::eye(4, 4);
	rootFrame.col(3).rows(0, 2) = arma::colvec({0, 0, -minZ + .0});
	m_privData->setKinematicModel(this, rootFrame, rootNode);

	m_privData->offsetAllBodies(tree);
}


dSpaceID PhysicsEnvironment::getCollisionSpaceID()
{
	return m_privData->m_collisionSpace.id();
}

dSpaceID PhysicsEnvironment::getVisualsSpaceID()
{
	return m_privData->m_visualSpace.id();
}

dWorldID PhysicsEnvironment::getWorldID()
{
	return m_privData->m_world.id();
}


void PhysicsEnvironment::simulateStep(Second step)
{
	pauseSimulation();
	m_privData->performStep(step.value());

	// call each callback after each step
	for (PhysicsEnvironmentStepCallback *callback : m_stepCallbacks)
	{
		callback->simulatorCallback(step.value());
	}
	unPauseSimulation();
}

void PhysicsEnvironment::pauseSimulation()
{
	m_privData->m_mutex.lock();
}

void PhysicsEnvironment::unPauseSimulation()
{
	m_privData->m_mutex.unlock();
}


void PhysicsEnvironment::addStepCallback(PhysicsEnvironmentStepCallback *callback)
{
	m_stepCallbacks.push_back(callback);
}


void PhysicsEnvironment::addMotor(MotorID id, ODEMotor *motor)
{
	m_motors[id] = motor;
}

}
