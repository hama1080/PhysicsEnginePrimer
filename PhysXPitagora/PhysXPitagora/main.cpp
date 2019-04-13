#include <iostream>
#include "PxPhysicsAPI.h"
#include "stl_output.h"

using namespace std;
using namespace physx;

PxDefaultAllocator      gAllocator;
PxDefaultErrorCallback  gErrorCallback;
PxFoundation*           gFoundation = NULL;
PxPhysics*              gPhysics = NULL;
PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene*                gScene = NULL;
PxPvd*                  gPvd = NULL;

PxRigidDynamic* gPusher = NULL;

// PhysX�̏�����
void initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	// PVD�̐ݒ�
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("localhost", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	// Scene�̍쐬
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);          // Right-hand coordinate system, Y-UP.
	gDispatcher = PxDefaultCpuDispatcherCreate(0);         // The number of worker threads is one.
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);

	// PVD�̐ݒ�
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
}

// Dynamic Rigidbody�̍쐬
PxRigidDynamic* createDynamic(const PxTransform& t,
	const PxGeometry& geometry, PxMaterial& material, PxReal density = 10.0f)
{
	PxRigidDynamic* rigid_dynamic
		= PxCreateDynamic(*gPhysics, t, geometry, material, density);
	gScene->addActor(*rigid_dynamic);
	return rigid_dynamic;
}

// Static Rigidbody�̍쐬
PxRigidStatic* createStatic(const PxTransform& t,
	const PxGeometry& geometry, PxMaterial& material)
{
	PxRigidStatic* rigid_static = PxCreateStatic(*gPhysics, t, geometry, material);
	gScene->addActor(*rigid_static);
	return rigid_static;
}


// �V�~�����[�V�����X�e�b�v��i�߂�
void stepPhysics()
{
	const PxReal kElapsedTime = 1.0f / 60.0f; // 60Hz
	gScene->simulate(kElapsedTime);
	gScene->fetchResults(true);
}

void createPitagoraScene()
{
	// �Ö��C�W���A�����C�W���A�����W���̏�
	PxMaterial* material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	////// �s�^�S�����u�̃t�B�[���h���쐬(static rigid body)
	// base plate(12m x 0.2m x 10m)
	const PxVec3 kPlateHalf(6.0f, 0.1f, 5.0f);
	createStatic(PxTransform(PxVec3(kPlateHalf.x, 0.0f, kPlateHalf.z)),
		PxBoxGeometry(kPlateHalf), *material);

	// �i��0
	const PxVec3 kStepHalf0(2.0f, 0.5f, 5.0f);
	createStatic(PxTransform(
		PxVec3(
			kPlateHalf.x * 2 - kStepHalf0.x,
			kPlateHalf.y + kStepHalf0.y,
			kStepHalf0.z)
	), PxBoxGeometry(kStepHalf0), *material);

	// �i��1
	const PxVec3 kStepHalf1(4.0f, 0.5f, 1.0f);
	createStatic(PxTransform(
		PxVec3(
			kStepHalf1.x,
			kPlateHalf.y + kStepHalf1.y,
			kStepHalf1.z)
	), PxBoxGeometry(kStepHalf1), *material);

	// �i��2
	const PxVec3 kStepHalf2(0.3f, 0.5f, 1.0f);
	createStatic(PxTransform(
		PxVec3(
			kStepHalf2.x,
			kPlateHalf.y + kStepHalf1.y * 2 + kStepHalf2.y,
			kStepHalf2.z)
	), PxBoxGeometry(kStepHalf2), *material);

	// slope
	const PxVec3 kSlopeHalf(3.7f, 0.1f, 1.0f);
	const PxReal kSlopeAngle = -PxPi / 36.0f; // 5 degree
	createStatic(PxTransform(
		PxVec3(
			kStepHalf2.x * 2 + kSlopeHalf.x,
			kPlateHalf.y + kStepHalf1.y * 2 + kStepHalf2.y,
			kSlopeHalf.z),
		PxQuat(kSlopeAngle, PxVec3(0.0f, 0.0f, 1.0f))
	), PxBoxGeometry(kSlopeHalf), *material);

	////// �����쐬(dynamic rigid body)
	const PxReal kSphereR = 0.25f;
	PxRigidDynamic* sphere = createDynamic(
		PxTransform(
			PxVec3(
				kSphereR,
				kPlateHalf.y + kStepHalf1.y * 2 + kStepHalf2.y * 2 + kSphereR,
				kStepHalf2.z)
		), PxSphereGeometry(kSphereR), *material);

	///// �����������̂��쐬(kinematic actor)
	const PxVec3 kPusherHalf(0.5f, 0.05f, 0.2f);
	gPusher = createDynamic(PxTransform(
		PxVec3(
			-kPusherHalf.x * 1.5,
			kPlateHalf.y + kStepHalf1.y * 2 + kStepHalf2.y * 2 + kSphereR,
			kStepHalf1.z)
	), PxBoxGeometry(kPusherHalf), *material);
	gPusher->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	///// �h�~�m���쐬
	const PxU32 kDominoCnt = 20;
	const PxBoxGeometry kDominoGeometry(0.05f, 0.5f, 0.2f);
	const PxReal kCircleR = kStepHalf0.x * 1.5f;
	const PxVec3 kCircleCenter(
		kStepHalf1.x * 2 + kDominoGeometry.halfExtents.x,
		kPlateHalf.y + kStepHalf0.y * 2 + kDominoGeometry.halfExtents.y,
		kCircleR + kStepHalf1.z);
	const PxReal kSplitAngle = PxPi / (kDominoCnt + 1);

	for (PxU32 i = 0; i != kDominoCnt; i++) {
		const PxVec3 dominoPos = kCircleCenter
			+ kCircleR * PxVec3(PxSin(kSplitAngle*i), 0.0f, -PxCos(kSplitAngle*i));

		createDynamic(
			PxTransform(dominoPos, PxQuat(-kSplitAngle * i, PxVec3(0.0f, 1.0f, 0.0f))),
			kDominoGeometry, *material);
	}

	///// �U��q���쐬
	const PxVec3 kChainCenter
		= kCircleCenter + kCircleR * PxVec3(0.0f, 0.0f, 1.0f) + PxVec3(-3.0f, 0.0f, 0.0f);
	const PxReal kChainLength = 5.0f;
	const PxU32 kChainCnt = 18;
	const PxReal kHookHalfHeight = 0.1f;
	const PxReal kElementR = 0.15f;
	const PxReal kLastElementR = kElementR * 3.0f;
	const PxReal kChainAngle = PxPi / 6.0f; // 30 degree

	// �U��q�̃t�b�N���쐬(static rigid body)
	PxRigidActor* chain_hook = createStatic(
		PxTransform(kChainCenter + PxVec3(0, kChainLength, 0)),
		PxBoxGeometry(0.5f, kHookHalfHeight, 0.1f), *material);

	PxRigidActor *actor0, *actor1;
	actor0 = chain_hook;
	for (PxU32 i = 0; i != kChainCnt; i++) {
		PxReal elementPosFromHook;
		PxSphereGeometry sphere;
		if (i < kChainCnt - 1) {
			elementPosFromHook = (kHookHalfHeight + kElementR) + (kElementR * 2)*i;
			sphere = PxSphereGeometry(kElementR);
		}
		else { // �Ō�̗v�f�̔��a��傫��
			elementPosFromHook = (kHookHalfHeight + kElementR)
				+ (kElementR * 2) * (i - 1) + (kElementR + kLastElementR);
			sphere = PxSphereGeometry(kLastElementR);
		}

		PxVec3 elementPos = kChainCenter
			+ PxVec3(
				elementPosFromHook * PxSin(kChainAngle),
				kChainLength - elementPosFromHook * PxCos(kChainAngle),
				0.0f);

		PxRigidDynamic* element = createDynamic(PxTransform(
			elementPos,
			PxQuat(
				PxHalfPi,
				PxVec3(0.0f, 0.0f, 1.0f)) * PxQuat(kChainAngle, PxVec3(0.0f, 0.0f, 1.0f))
		), sphere, *material, 1.0f);

		//position iteration count�̐ݒ�
		element->setSolverIterationCounts(64, 1);
		element->putToSleep();  // actor���X���[�v������
		actor1 = element;

		PxReal jointPosFromHook = kHookHalfHeight + (kElementR * 2) * i;
		PxVec3 jointPos = kChainCenter
			+ PxVec3(
				jointPosFromHook * PxSin(kChainAngle),
				kChainLength - jointPosFromHook * PxCos(kChainAngle),
				0.0f);

		PxSphericalJoint* joint = PxSphericalJointCreate(
			*gPhysics,
			actor0,
			PxTransform(
				actor0->getGlobalPose().q.rotateInv(
					PxVec3(jointPos - actor0->getGlobalPose().p)
				),
				PxQuat(-PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f))),
			actor1,
			PxTransform(
				actor1->getGlobalPose().q.rotateInv(
					PxVec3(jointPos - actor1->getGlobalPose().p)
				),
				PxQuat(-PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)))
		);
		actor0 = element;
	}

	///// �\�������쐬
	const PxVec3 kStructureCenter
		= PxVec3(kChainCenter.x, kPlateHalf.y, kChainCenter.z)
		+ PxVec3(-3.0f, 0.0f, -1.25f);  // �I�t�Z�b�g
	const PxU32 kStructureCnt = 7;
	const PxReal kStructureLength = 0.2f;

	for (PxU32 x = 0; x != kStructureCnt; x++) {
		for (PxU32 y = 0; y != kStructureCnt; y++) {
			for (PxU32 z = 0; z != kStructureCnt; z++) {
				const PxVec3 kElementPos = kStructureCenter
					+ PxVec3(
						kStructureLength * 2 * x,
						kStructureLength + kStructureLength * 2 * y,
						kStructureLength * 2 * z);

				PxRigidDynamic* element = createDynamic(
					PxTransform(kElementPos),
					PxBoxGeometry(kStructureLength, kStructureLength, kStructureLength),
					*material, 0.01f); // ���₷�����邽�߂Ɍy������

				element->putToSleep();
			}
		}
	}
}

int main(void)
{
	initPhysics();
	cout << "PhysXPitagora" << endl;
	cout << "Start simulation" << endl;

	const PxU32 kMaxSimulationStep = 1000;

	createPitagoraScene();

	for (PxU32 step = 0; step != kMaxSimulationStep; step++) {
		if (step < 100) {
			PxVec3 pusher_pos = gPusher->getGlobalPose().p;
			gPusher->setKinematicTarget(
				PxTransform(pusher_pos + PxVec3(0.01f, 0.0f, 0.0f)));
		}
		stepPhysics();
	}
	cout << "End simulation" << endl;

	// STL�t�@�C���̏����o��
	/*
	PxActorTypeFlags desired_types
		= PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC;
	PxU32 actor_cnt = gScene->getNbActors(desired_types);
	PxActor** actor_buffer = new PxActor*[actor_cnt];
	gScene->getActors(desired_types, actor_buffer, actor_cnt);
	
	StlOutput stl_output;
	stl_output.outputStl("F:/stl/", actor_buffer, actor_cnt, true);
	*/
	
	int tmp;
	cin >> tmp;
	return 0;
}