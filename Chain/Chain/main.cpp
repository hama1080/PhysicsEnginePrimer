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
	PxInitExtensions(*gPhysics, gPvd);

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
	cout << "Chain" << endl;
	cout << "Start physics process" << endl;

	PxMaterial* material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);	//static friction, dynamic friction, restitution

	const float kHeight = 10.0f;
	const float kHookHalfHeight = 0.1f;
	// create hook as static rigid body
	PxRigidActor* hook = createStatic(PxTransform(PxVec3(0, kHeight, 0)), PxBoxGeometry(0.5f, kHookHalfHeight, 0.1f), *material);
	
	// create chain
	PxRigidActor *actor0, *actor1;
	actor0 = hook;
	const float kElementRadius = 0.1f;
	const float kElementHeight = 0.2f;
	for (int i = 0; i != 50; i++) {
		PxRigidDynamic* element = createDynamic(
			PxTransform(PxVec3(0.0f, kHeight - (kElementHeight + kElementRadius + kHookHalfHeight) - ((kElementHeight + kElementRadius) * 2) * i, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))),
			PxCapsuleGeometry(kElementRadius, kElementHeight), *material);
		element->setSolverIterationCounts(10,1);	// set position iteration count
		actor1 = element;


		PxVec3 joint_pos(0.0f, kHeight - kHookHalfHeight - ((kElementHeight + kElementRadius) * 2) * i, 0.0f);
		PxSphericalJoint* joint = PxSphericalJointCreate(*gPhysics,
			actor0,
			PxTransform(actor0->getGlobalPose().q.rotateInv(PxVec3(joint_pos - actor0->getGlobalPose().p)), PxQuat(-PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))),
			actor1,
			PxTransform(actor1->getGlobalPose().q.rotateInv(PxVec3(joint_pos - actor1->getGlobalPose().p)), PxQuat(-PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)))
		);

		actor0 = element;
	}

	// simulation loop
	for (PxU32 i = 0; i != 1000; i++)
	{
		if(i < 200)
			static_cast<PxRigidDynamic*>(actor1)->addForce(PxVec3(10, 0, 0));

		if(i % 100 == 0)
			cout << "Simulation step: " << i << endl;
		stepPhysics();
	}

	cout << "End physics process." << endl;
	int tmp;
	cin >> tmp;
	return 0;
}
