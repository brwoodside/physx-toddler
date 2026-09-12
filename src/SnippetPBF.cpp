// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates fluid simulation using position-based dynamics
// particle simulation. It creates a container and drops a body of water.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "extensions/PxParticleExt.h"

using namespace physx;
using namespace ExtGpu;

static PxDefaultAllocator				gAllocator;
static PxDefaultErrorCallback			gErrorCallback;
static PxFoundation*					gFoundation			= NULL;
static PxPhysics*						gPhysics			= NULL;
static PxDefaultCpuDispatcher*			gDispatcher			= NULL;
static PxScene*							gScene				= NULL;
static PxMaterial*						gMaterial			= NULL;
static PxPBDParticleSystem*				gParticleSystem		= NULL;
static PxParticleAndDiffuseBuffer*		gParticleBuffer 	= NULL;
static bool								gIsRunning			= true;
static bool								gStep				= true;


static int								gMaxDiffuseParticles = 0;

// -----------------------------------------------------------------------------------------------------------------
static void initScene()
{
	PxCudaContextManager* cudaContextManager = NULL;
	if (PxGetSuggestedCudaDeviceOrdinal(gFoundation->getErrorCallback()) >= 0)
	{
		// initialize CUDA
		PxCudaContextManagerDesc cudaContextManagerDesc;
		cudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
		if (cudaContextManager && !cudaContextManager->contextIsValid())
		{
			cudaContextManager->release();
			cudaContextManager = NULL;
		}
	}
	if (cudaContextManager == NULL)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Failed to initialize CUDA!\n");
	}

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cudaContextManager = cudaContextManager;
	sceneDesc.staticStructure = PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.solverType = PxSolverType::eTGS;
	gScene = gPhysics->createScene(sceneDesc);
}

int getNumDiffuseParticles()
{
	return gMaxDiffuseParticles;
}

// -----------------------------------------------------------------------------------------------------------------
static void initParticles(const PxU32 numX, const PxU32 numY, const PxU32 numZ, const PxVec3& position = PxVec3(0, 0, 0), const PxReal particleSpacing = 0.2f, const PxReal fluidDensity = 1000.f, const PxU32 maxDiffuseParticles = 100000)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return;

	const PxU32 maxParticles = numX * numY * numZ;

	const PxReal restOffset = 0.5f * particleSpacing / 0.6f;
	
	// Material setup
	PxPBDMaterial* defaultMat = gPhysics->createPBDMaterial(0.05f, 0.05f, 0.f, 0.001f, 0.5f, 0.005f, 0.01f, 0.f, 0.f);

	defaultMat->setViscosity(0.001f);
	defaultMat->setSurfaceTension(0.00704f);
	defaultMat->setCohesion(0.0704f);
	defaultMat->setVorticityConfinement(10.f);

	PxPBDParticleSystem *particleSystem = gPhysics->createPBDParticleSystem(*cudaContextManager, 96);
	gParticleSystem = particleSystem;

	// General particle system setting
	
	const PxReal solidRestOffset = restOffset;
	const PxReal fluidRestOffset = restOffset * 0.6f;
	const PxReal particleMass = fluidDensity * 1.333f * 3.14159f * particleSpacing * particleSpacing * particleSpacing;
	particleSystem->setRestOffset(restOffset);
	particleSystem->setContactOffset(restOffset + 0.01f);
	particleSystem->setParticleContactOffset(fluidRestOffset / 0.6f);
	particleSystem->setSolidRestOffset(solidRestOffset);
	particleSystem->setFluidRestOffset(fluidRestOffset);
	particleSystem->enableCCD(false);
	particleSystem->setMaxVelocity(solidRestOffset*100.f);

	gScene->addActor(*particleSystem);
	
	// Diffuse particles setting
	PxDiffuseParticleParams dpParams;
	dpParams.threshold = 300.0f;
	dpParams.bubbleDrag = 0.9f;
	dpParams.buoyancy = 0.9f;
	dpParams.airDrag = 0.0f;
	dpParams.kineticEnergyWeight = 0.01f;
	dpParams.pressureWeight = 1.0f;
	dpParams.divergenceWeight = 10.f;
	dpParams.lifetime = 1.0f;
	dpParams.useAccurateVelocity = false;

	gMaxDiffuseParticles = maxDiffuseParticles;

	// Create particles and add them to the particle system
	const PxU32 particlePhase = particleSystem->createPhase(defaultMat, PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

	PxU32* phase = cudaContextManager->allocPinnedHostBuffer<PxU32>(maxParticles);
	PxVec4* positionInvMass = cudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);
	PxVec4* velocity = cudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);

	PxReal x = position.x;
	PxReal y = position.y;
	PxReal z = position.z;

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numY; ++j)
		{
			for (PxU32 k = 0; k < numZ; ++k)
			{
				const PxU32 index = i * (numY * numZ) + j * numZ + k;

				PxVec4 pos(x, y, z, 1.0f / particleMass);
				phase[index] = particlePhase;
				positionInvMass[index] = pos;
				velocity[index] = PxVec4(0.0f);

				z += particleSpacing;
			}
			z = position.z;
			y += particleSpacing;
		}
		y = position.y;
		x += particleSpacing;
	}


	ExtGpu::PxParticleAndDiffuseBufferDesc bufferDesc;
	bufferDesc.maxParticles = maxParticles;
	bufferDesc.numActiveParticles = maxParticles;
	bufferDesc.maxDiffuseParticles = maxDiffuseParticles;
	bufferDesc.maxActiveDiffuseParticles = maxDiffuseParticles;
	bufferDesc.diffuseParams = dpParams;

	bufferDesc.positions = positionInvMass;
	bufferDesc.velocities = velocity;
	bufferDesc.phases = phase;

	gParticleBuffer = physx::ExtGpu::PxCreateAndPopulateParticleAndDiffuseBuffer(bufferDesc, cudaContextManager);
	gParticleSystem->addParticleBuffer(gParticleBuffer);

	cudaContextManager->freePinnedHostBuffer(positionInvMass);
	cudaContextManager->freePinnedHostBuffer(velocity);
	cudaContextManager->freePinnedHostBuffer(phase);
}

PxPBDParticleSystem* getParticleSystem()
{
	return gParticleSystem;
}

PxParticleAndDiffuseBuffer* getParticleBuffer()
{
	return gParticleBuffer;
}




// ---------------------------------------------------------------- Magna-Tile ball run scene
// World is metric: a 1.2 m wide x 1.4 m tall run inside a 0.24 m deep glass channel.
static const PxReal kW = 1.2f, kH = 1.4f, kDepth = 0.12f;   // kDepth = half depth of the channel
static const PxReal kTileHalfT = 0.006f, kBallR = 0.035f;
struct Seg { PxReal x0, y0, x1, y1; };
static const Seg kTiles[] = {
	{0.05f, 1.15f, 0.75f, 1.02f},  // ramp 1 (slopes right)
	{1.15f, 0.92f, 0.45f, 0.80f},  // ramp 2 (slopes left)
	{0.05f, 0.70f, 0.75f, 0.58f},  // ramp 3
	{1.15f, 0.48f, 0.45f, 0.36f},  // ramp 4
	{0.30f, 0.00f, 0.30f, 0.18f}, {0.90f, 0.00f, 0.90f, 0.18f},  // basin walls
	{0.55f, 0.20f, 0.65f, 0.20f},  // bump
};
static const PxVec3 kSpout(0.20f, 1.36f, 0.0f);
static PxU32 gFrame = 0;
static const PxU32 kMaxBalls = 30;
static std::vector<PxRigidDynamic*> gBalls;
static PxReal gBallRadius = kBallR;

static void addTile(const Seg& s)
{
	const PxVec3 a(s.x0, s.y0, 0), b(s.x1, s.y1, 0);
	const PxVec3 d = b - a;
	const PxReal len = d.magnitude();
	const PxReal ang = atan2f(d.y, d.x);
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.5f * len, kTileHalfT, kDepth), *gMaterial);
	shape->setContactOffset(0.004f);
	shape->setRestOffset(0.0f);
	PxRigidStatic* body = gPhysics->createRigidStatic(PxTransform(0.5f * (a + b), PxQuat(ang, PxVec3(0, 0, 1))));
	body->attachShape(*shape);
	gScene->addActor(*body);
	shape->release();
}

// ---- ball management API (also used by the render-side UI overlay)
PxRigidDynamic* spawnBall(const PxVec3& pos, PxReal r)
{
	if (gBalls.size() >= kMaxBalls) return NULL;
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(r), *gMaterial);
	shape->setContactOffset(0.004f);
	shape->setRestOffset(0.0f);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(pos));
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, 300.0f);   // light plastic, ~54 g at r = 35 mm
	body->setSolverIterationCounts(8, 2);
	gScene->addActor(*body);
	shape->release();
	gBalls.push_back(body);
	return body;
}

void dropBall()
{
	spawnBall(kSpout + PxVec3(0.10f * (gBalls.size() % 3), 0, 0), gBallRadius);
}

void clearBalls()
{
	for (size_t i = 0; i < gBalls.size(); ++i)
	{
		gScene->removeActor(*gBalls[i]);
		gBalls[i]->release();
	}
	gBalls.clear();
}

PxU32  getNumBalls()               { return PxU32(gBalls.size()); }
PxU32  getMaxBalls()               { return kMaxBalls; }
PxReal getBallRadius()             { return gBallRadius; }
void   setBallRadius(PxReal r)     { gBallRadius = PxClamp(r, 0.015f, 0.08f); }
bool   isPaused()                  { return !gIsRunning; }
void   setPaused(bool p)           { gIsRunning = !p; }
void   getRunExtent(PxReal& w, PxReal& h, PxReal& halfDepth) { w = kW; h = kH; halfDepth = kDepth; }

// -----------------------------------------------------------------------------------------------------------------
void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL);

	initScene();
	if (PxCudaContextManager* ccm = gScene->getCudaContextManager())
		printf("PhysX GPU: %s\n", ccm->getDeviceName());
	gMaterial = gPhysics->createMaterial(0.4f, 0.3f, 0.35f);

	// Water: a block above the first ramp, spacing 12 mm, ~6000 fluid particles.
	initParticles(21, 17, 19, PxVec3(0.05f, 1.18f, -kDepth + 0.015f), 0.012f, 1000.0f, 20000);
	gParticleSystem->setMaxVelocity(8.0f);

	// Container: floor, side walls, and two "glass" walls that make the channel thin in z.
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.0f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(1.f, 0.f, 0.f, 0.0f), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(-1.f, 0.f, 0.f, kW), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, 1.f, kDepth), *gMaterial));
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 0.f, -1.f, kDepth), *gMaterial));
	for (PxU32 i = 0; i < sizeof(kTiles) / sizeof(kTiles[0]); ++i)
		addTile(kTiles[i]);
	dropBall();
	printf("Ball run: %u tiles, %u water particles. Keys: B = drop ball, M = place mode, X = clear balls, [ ] = ball radius, P = pause, O = step, C = reset view\n"
		"Mouse: left drag = orbit, right drag = pan, wheel / middle drag = zoom\n",
		PxU32(sizeof(kTiles) / sizeof(kTiles[0])), 21u * 17u * 19u);
}

// ---------------------------------------------------
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning || gStep)
	{
		gStep = false;
		const PxReal dt = 1.0f / 60.0f;
		++gFrame;
		gScene->simulate(dt);
		gScene->fetchResults(true);
		gScene->fetchResultsParticleSystem();
	}
}

void cleanupPhysics(bool /*interactive*/)
{
	PxCudaContextManager* ccm = gScene ? gScene->getCudaContextManager() : NULL;
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (ccm) ccm->release();
	PX_RELEASE(gFoundation);
	printf("SnippetPBF (ball run) done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	switch(toupper(key))
	{
	case 'P':	gIsRunning = !gIsRunning;	break;
	case 'O':	gIsRunning = false; gStep = true;	break;
	case 'B':	dropBall(); break;
	case 'X':	clearBalls(); break;
	case '[':	setBallRadius(gBallRadius - 0.005f); break;
	case ']':	setBallRadius(gBallRadius + 0.005f); break;
#ifdef RENDER_SNIPPET
	case 'C':	{ extern void resetCamera(); resetCamera(); } break;
	case 'M':	{ extern void togglePlaceMode(); togglePlaceMode(); } break;
#endif
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif
	return 0;
}
