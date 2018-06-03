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

// Proceed the step of physics environment
void stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

int main(void)
{
	initPhysics();
	cout << "Free Fall" << endl;
	cout << "Start physics process" << endl;

	PxMaterial* material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);	//static friction, dynamic friction, restitution

	// create plate as kinematic rigid body
	PxRigidDynamic* plate = createDynamic(PxTransform(PxVec3(0)), PxBoxGeometry(10.0f, 0.1f, 10.0f), *material);
	plate->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	// drop various rigid bodies
	createDynamic(PxTransform((PxVec3(1.5f, 11.5f, 0.0f))), PxSphereGeometry(0.3f), *material);
	createDynamic(PxTransform((PxVec3(0.0f, 10.0f, 0.0f))), PxSphereGeometry(0.5f), *material);
	createDynamic(PxTransform((PxVec3(-0.5f, 8.5f, 0.0f))), PxSphereGeometry(0.7f), *material);

	createDynamic(PxTransform((PxVec3(-1.5f, 9.5f, 0.0f))), PxBoxGeometry(0.7f, 0.5f, 0.5f), *material);
	createDynamic(PxTransform((PxVec3(1.5f, 10.5f, 0.0f))), PxBoxGeometry(0.5f, 0.5f, 0.5f), *material);
	createDynamic(PxTransform((PxVec3(0.5f, 11.5f, 0.0f))), PxBoxGeometry(0.5f, 0.7f, 0.5f), *material);

	for (PxU32 i = 0; i != 500; i++)
	{
		if(i % 100 == 0)
			cout << "Simulation step: " << i << endl;
		stepPhysics();
	}

	cout << "End physics process." << endl;
	int tmp;
	cin >> tmp;
	return 0;
}
