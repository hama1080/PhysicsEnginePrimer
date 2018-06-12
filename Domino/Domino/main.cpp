#include <iostream>
#include "PxPhysicsAPI.h"

using namespace std;
using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics = NULL;
PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene = NULL;
PxPvd*                  gPvd = NULL;

// Initialize PhysX
void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	// PVD(PhysX Visual Debugger) setting. To use PVD, we need to build the project as Debug mode.
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	// Scene setting
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);			// Right-hand coordinate system, Y-UP.
	gDispatcher = PxDefaultCpuDispatcherCreate(1);			// The number of worker threads is one.
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	// PVD setting
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

// Create Dynamic Rigidbody
PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, PxMaterial& material)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, material, 10.0f);
	gScene->addActor(*dynamic);
	return dynamic;
}

// Create Static Rigidbody
PxRigidStatic* createStatic(const PxTransform& t, const PxGeometry& geometry, PxMaterial& material)
{
	PxRigidStatic* static_actor = PxCreateStatic(*gPhysics, t, geometry, material);
	gScene->addActor(*static_actor);
	return static_actor;
}

// Proceed the step of physics environment
void stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

int main(void)
{
	initPhysics();
	cout << "Domino" << endl;
	cout << "Start physics process" << endl;

	PxMaterial* material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);	//static friction, dynamic friction, restitution

	// create plate as static rigid body
	createStatic(PxTransform(PxVec3(0)), PxBoxGeometry(10.0f, 0.1f, 10.0f), *material);
	
	// create domino
	const unsigned int kDominoCnt = 20;
	for (unsigned int i = 0; i != kDominoCnt; i++)
	{
		createDynamic(PxTransform((PxVec3(9.5f -i*0.5, 0.5f, 0.0f))), PxBoxGeometry(0.05f, 0.5f, 0.2f), *material);
	}

	// create pusher(kinematic actor)
	PxRigidDynamic* pusher = createDynamic(PxTransform((PxVec3(10 + 0.5f, 1.0f, 0.0f))), PxBoxGeometry(0.5f, 0.05f, 0.2f), *material);
	pusher->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	// simulation loop
	for (PxU32 i = 0; i != 500; i++)
	{
		// push!
		if (i < 100) {
			PxTransform next_position(pusher->getGlobalPose().p - PxVec3(0.01f, 0.0f, 0.0f));
			pusher->setKinematicTarget(next_position);
		}

		if(i % 100 == 0)
			cout << "Simulation step: " << i << endl;
		stepPhysics();
	}

	cout << "End physics process." << endl;
	int tmp;
	cin >> tmp;
	return 0;
}
