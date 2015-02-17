/*
 * physicsVisualization.cpp
 *
 *  Created on: 26.05.2014
 *      Author: lutz
 */

#include "physicsVisualization.h"
#include <thread>

#include <drawstuff/drawstuff.h>

#include "odeUserObject.h"
#include "kinematicEngine/utils/utils.h"

namespace kinematicEngine {

PhysicsVisualization::PhysicsVisualization()
{
}

PhysicsVisualization::~PhysicsVisualization() {
}


#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#define dsDrawCylinder dsDrawCylinderD
#endif

//#define DRAW_JOINTS_TOO

static dsFunctions g_fn;
static PhysicsEnvironment *g_envToDraw;
static std::thread *g_drawingThread;

PhysicsVisualization& PhysicsVisualization::getInstance()
{
	static PhysicsVisualization* instance(nullptr);
	if (nullptr == instance) {
		instance = new PhysicsVisualization();
	}

	return *instance;
}


static void drawFunctionWrapper(int pause)
{
	PhysicsVisualization &instance = PhysicsVisualization::getInstance();
	instance.draw(pause);
}

static void start()
{
    // initial camera position
    static float xyz[3] = {1, 1, 0.5};
    static float hpr[3] = {225, 0, 0};
    dsSetViewpoint (xyz,hpr);
}
static void stop()
{
	exit(0);
}


static void drawGeom(dGeomID geomID)
{
//	printf("1%lu", (uint64_t)geomID);
    int gclass = dGeomGetClass(geomID);
//	printf("2\n");
    const dReal *pos = NULL;
    const dReal *rot = NULL;
    bool canDrawJoints = false;

    ODEUserObject* userObj = (ODEUserObject*)dGeomGetData(geomID);
    if (nullptr != userObj) {
        dsSetColorAlpha(1, 1, 1, 1);
//        printf("%f %f %f %f\n", userObj->colorVec[0], userObj->colorVec[1], userObj->colorVec[2], userObj->colorVec[3]);
        dsSetTexture (userObj->textureNum);
        dsSetColorAlpha(userObj->m_colorVec[0], userObj->m_colorVec[1], userObj->m_colorVec[2], userObj->m_colorVec[3]);
    } else {
        dsSetColorAlpha(1, 1, 0, 1);
        dsSetTexture (DS_WOOD);
    }

    switch (gclass) {
        case dSphereClass:
			if (nullptr != userObj && userObj->visible) {
				pos = dGeomGetPosition(geomID);
				rot = dGeomGetRotation(geomID);
				dsDrawSphere(pos, rot, dGeomSphereGetRadius(geomID));
			}
            canDrawJoints = true;
            break;
        case dBoxClass:
        {
			if (nullptr != userObj && userObj->visible) {
				pos = dGeomGetPosition(geomID);
				rot = dGeomGetRotation(geomID);
				dVector3 lengths;
				dGeomBoxGetLengths(geomID, lengths);
				dsDrawBox(pos, rot, lengths);
			}
            canDrawJoints = true;
            break;
        }
        case dCylinderClass:
        {
			if (nullptr != userObj && userObj->visible) {
				pos = dGeomGetPosition(geomID);
				rot = dGeomGetRotation(geomID);
				dReal length;
				dReal radius;
                dGeomCylinderGetParams(geomID, &radius, &length);
                dsDrawCylinder(pos, rot, length, radius);
			}
            canDrawJoints = true;
            break;
        }

        default:
			break;
    }
//	printf("class: %d\n", gclass);

    if (canDrawJoints) {
#ifdef DRAW_JOINTS_TOO
        dBodyID body = dGeomGetBody(geomID);
        int numJoints = dBodyGetNumJoints(body);
        for (int i = 0; i < numJoints; ++i)
        {
			dJointID joint = dBodyGetJoint(body, i);
			int jointClass = dJointGetType(joint);
			switch (jointClass)
			{
				case dJointTypeHinge:
				{
					dVector3 a11;
					dBodyID body1 = dJointGetBody(joint, 0);
					dBodyID body2 = dJointGetBody(joint, 1);

					if (body1 && body2) {
						const dReal* bodyPos1 =  dBodyGetPosition(body1);
						const dReal* bodyPos2 =  dBodyGetPosition(body2);
						dJointGetHingeAnchor(joint, a11);

						dsSetColor(1, 0, 0);
						dsDrawLine(a11, bodyPos1);
						dsDrawLine(a11, bodyPos2);

						dsSetColor(0, 1, 0);
						dsDrawLine(bodyPos1, bodyPos2);
					}
				}
			}
        }
#endif
    }
}


void PhysicsVisualization::setEnvironmentToDraw(PhysicsEnvironment *env) {
	// dummy implementation
	std::unique_lock<std::mutex> lock(m_mutex);
	if (nullptr != g_envToDraw && nullptr != g_drawingThread)
	{
		// quit current simulation
		dsStop();
		g_drawingThread->join();
		delete g_drawingThread;
		g_drawingThread = nullptr;
	}

	g_envToDraw = env;

	if (nullptr != g_envToDraw)
	{
		// setup drawing stuff
	    g_fn.version = DS_VERSION;
	    g_fn.start = &start;
	    g_fn.step = &drawFunctionWrapper;
	    g_fn.stop = &stop;
	    g_fn.command = 0;
	    g_fn.path_to_textures = "textures/";

	    g_drawingThread = new std::thread(dsSimulationLoop, 0, (char**)NULL, 640, 480, &g_fn);
	}
}

void PhysicsVisualization::draw(int pause)
{
	UNUSED(pause);
	if (nullptr != g_drawingThread && nullptr != g_envToDraw)
	{
		g_envToDraw->pauseSimulation();
	    // now we draw everything
		dSpaceID collisionSpace = g_envToDraw->getCollisionSpaceID();
		dSpaceID visualsSpace = g_envToDraw->getVisualsSpaceID();

	    uint ngeoms = dSpaceGetNumGeoms(collisionSpace);
	    for (uint i = 0; i < ngeoms; ++i) {
	        dGeomID g = dSpaceGetGeom(collisionSpace, i);
	        drawGeom(g);
	    }

	    ngeoms = dSpaceGetNumGeoms(visualsSpace);
		for (uint i = 0; i < ngeoms; ++i) {
			dGeomID g = dSpaceGetGeom(visualsSpace, i);
			drawGeom(g);
		}
		g_envToDraw->unPauseSimulation();
	}
}

}
