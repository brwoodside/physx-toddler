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
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
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
static PxU32 gParticleCapacity = 0;
static PxReal gParticleInvMass = 1.0f;
static PxU32 gParticlePhase = 0;

// Creates the fluid system with room for `capacity` particles, none active.  The hose fills it at runtime.
static void initParticles(const PxU32 capacity, const PxReal particleSpacing, const PxReal fluidDensity, const PxU32 maxDiffuseParticles)
{
	PxCudaContextManager* cudaContextManager = gScene->getCudaContextManager();
	if (cudaContextManager == NULL)
		return;

	const PxU32 maxParticles = capacity;
	gParticleCapacity = capacity;

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

	gParticlePhase = particlePhase;
	gParticleInvMass = 1.0f / particleMass;
	PxU32* phase = cudaContextManager->allocPinnedHostBuffer<PxU32>(maxParticles);
	PxVec4* positionInvMass = cudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);
	PxVec4* velocity = cudaContextManager->allocPinnedHostBuffer<PxVec4>(maxParticles);
	for (PxU32 i = 0; i < maxParticles; ++i)
	{
		phase[i] = particlePhase;
		positionInvMass[i] = PxVec4(0.6f, -5.0f, 0.0f, gParticleInvMass);   // parked; inactive until the hose emits it
		velocity[i] = PxVec4(0.0f);
	}

	ExtGpu::PxParticleAndDiffuseBufferDesc bufferDesc;
	bufferDesc.maxParticles = maxParticles;
	bufferDesc.numActiveParticles = 0;
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




// ---------------------------------------------------------------- Magna-Tiles ball run scene
// Modelled on the Lakeshore set photo: a tower of square tiles, half-pipe chutes zig-zagging down
// through ring tiles with wall tiles at the drops, a funnel on top and a catch bowl at the bottom.
// Built at 2x real size (tile = 150 mm, real 76 mm) so the 12 mm water particles resolve a chute.
static const PxReal T = 0.15f;                         // one tile
static const PxReal kTileT = 0.04f * T;                // tile thickness
static const PxReal kChuteR = 0.36f * T, kChuteWall = 0.03f * T;
static const PxReal kTrackZ = 0.5f * T;                // chute centre line, in front of the backdrop
static const PxReal kW = 4.3f * T, kH = 5.6f * T;      // extents used by the placement ghost
static const PxReal kBallR = 0.025f;                   // 50 mm ball at 2x scale
static const PxVec3 kSpout(0.5f * T, 5.9f * T, kTrackZ);
static PxU32 gFrame = 0;
static const PxU32 kMaxBalls = 30;
static std::vector<PxRigidDynamic*> gBalls;
static PxReal gBallRadius = kBallR;

// Actor userData tags the piece kind and colour for the renderer: (kind << 8) | colourIndex.
enum PieceKind { PIECE_TILE = 1, PIECE_CHUTE = 2 };
static PxU32 gNumPieces = 0;

static PxRigidStatic* gLastPiece = NULL;
static PxRigidStatic* newPiece(PieceKind kind, PxU32 colour, const PxTransform& pose = PxTransform(PxIdentity))
{
	PxRigidStatic* body = gPhysics->createRigidStatic(pose);
	gLastPiece = body;
	body->userData = reinterpret_cast<void*>(uintptr_t((PxU32(kind) << 8) | (colour & 0xff)));
	++gNumPieces;
	return body;
}

// Box panel with its local x / y axes given explicitly (z = x cross y), attached to `body`.
static void addPanel(PxRigidStatic* body, const PxVec3& centre, PxVec3 ax, PxVec3 ay, const PxVec3& half)
{
	ax.normalize();
	ay = (ay - ax * ax.dot(ay)).getNormalized();
	const PxVec3 az = ax.cross(ay);
	const PxQuat q(PxMat33(ax, ay, az));
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(half), *gMaterial);
	shape->setContactOffset(0.003f);
	shape->setRestOffset(0.0f);
	shape->setLocalPose(PxTransform(centre, q));
	body->attachShape(*shape);
	shape->release();
}

// Tile orientation: local x = up cross n, local y = up, local z = n (the tile's normal).
static PxQuat tileQuat(const PxVec3& n, const PxVec3& up)
{
	const PxVec3 ay = up.getNormalized(), ax = ay.cross(n.getNormalized()).getNormalized(), az = ax.cross(ay);
	return PxQuat(PxMat33(ax, ay, az));
}

// Square tile: centre, normal direction n, "up" direction in the tile plane.  The actor's pose is
// the tile centre so the editor can move it with setGlobalPose.
static void addSquareTile(const PxVec3& c, const PxVec3& n, const PxVec3& up, PxU32 colour)
{
	PxRigidStatic* body = newPiece(PIECE_TILE, colour, PxTransform(c, tileQuat(n, up)));
	addPanel(body, PxVec3(0), PxVec3(1, 0, 0), PxVec3(0, 1, 0), PxVec3(0.5f * T, 0.5f * T, 0.5f * kTileT));
	gScene->addActor(*body);
}

// Ring tile: square frame with an octagonal hole of radius `hole`, same orientation convention.
static void addRingTile(const PxVec3& c, const PxVec3& n, const PxVec3& up, PxReal hole, PxU32 colour)
{
	PxRigidStatic* body = newPiece(PIECE_TILE, colour, PxTransform(c, tileQuat(n, up)));
	const PxVec3 ax(1, 0, 0), ay(0, 1, 0);
	const PxReal band = 0.5f * (T - 2.0f * hole);                  // width of the four outer bands
	const PxVec3 hb(0.5f * T, 0.5f * band, 0.5f * kTileT);
	addPanel(body, ay * (0.5f * T - 0.5f * band), ax, ay, hb);       // top
	addPanel(body, -ay * (0.5f * T - 0.5f * band), ax, ay, hb);      // bottom
	addPanel(body, ax * (0.5f * T - 0.5f * band), ay, ax, hb);       // right
	addPanel(body, -ax * (0.5f * T - 0.5f * band), ay, ax, hb);      // left
	// four diagonal corner fillets turn the square hole into an octagon
	const PxReal d = hole * 0.9239f;                                 // apothem of the octagon
	const PxReal side = 2.0f * hole * 0.3827f;
	for (int i = 0; i < 4; ++i)
	{
		const PxReal sx = (i & 1) ? -1.0f : 1.0f, sy = (i & 2) ? -1.0f : 1.0f;
		const PxVec3 dir = (ax * sx + ay * sy).getNormalized();
		const PxVec3 tan = (ax * sx - ay * sy).getNormalized();
		addPanel(body, dir * (d + 0.06f * T), tan, dir, PxVec3(0.5f * side + 0.02f * T, 0.06f * T, 0.5f * kTileT));
	}
	gScene->addActor(*body);
}

// ---- tile editor API (used by the render-side build mode)
static const PxReal kRingHole = 0.42f * T;
PxRigidActor* createTile(const PxVec3& c, const PxVec3& n, const PxVec3& up, PxU32 colour, bool ring)
{
	if (ring) addRingTile(c, n, up, kRingHole, colour); else addSquareTile(c, n, up, colour);
	if (getenv("BALLRUN_TRACE")) printf("createTile %s colour %u at (%.3f %.3f %.3f)\n", ring ? "ring" : "square", colour, c.x, c.y, c.z);
	return gLastPiece;
}
bool isTile(PxRigidActor* a)               { return a && (PxU32(uintptr_t(a->userData)) >> 8) == PIECE_TILE; }
PxU32 getPieceColour(PxRigidActor* a)      { return PxU32(uintptr_t(a->userData)) & 0xff; }
void  getTileFrame(PxRigidActor* a, PxVec3& c, PxVec3& n, PxVec3& up)
{
	const PxTransform p = a->getGlobalPose();
	c = p.p; n = p.q.rotate(PxVec3(0, 0, 1)); up = p.q.rotate(PxVec3(0, 1, 0));
}
void  moveTile(PxRigidActor* a, const PxVec3& c)   { a->setGlobalPose(PxTransform(c, a->getGlobalPose().q)); if (getenv("BALLRUN_TRACE")) printf("moveTile -> (%.3f %.3f %.3f)\n", c.x, c.y, c.z); }
void  removeTile(PxRigidActor* a)                  { gScene->removeActor(*a); a->release(); --gNumPieces; if (getenv("BALLRUN_TRACE")) printf("removeTile\n"); }
PxReal getTileSize()                               { return T; }
PxReal getTileThickness()                          { return kTileT; }
// Closest tile under a world ray (chutes and balls block the ray but are not returned).
PxRigidActor* pickTile(const PxVec3& origin, const PxVec3& dir, PxVec3& hitPos)
{
	PxRaycastBuffer hit;
	if (gScene->raycast(origin, dir.getNormalized(), 50.0f, hit) && hit.hasBlock && isTile(hit.block.actor))
	{
		hitPos = hit.block.position;
		return hit.block.actor;
	}
	return NULL;
}

// Straight half-pipe chute from a to b (its axis), open at the top, 7 wall segments over 200 degrees.
static void addChute(const PxVec3& a, const PxVec3& b, PxU32 colour)
{
	PxRigidStatic* body = newPiece(PIECE_CHUTE, colour);
	const PxVec3 axis = (b - a).getNormalized();
	const PxReal len = (b - a).magnitude();
	const PxVec3 mid = 0.5f * (a + b);
	const PxVec3 side = axis.cross(PxVec3(0, 1, 0)).getNormalized();   // horizontal, across the pipe
	const PxVec3 down = side.cross(axis).getNormalized() * -1.0f;      // perpendicular "down"
	const int nseg = 7; const PxReal span = 200.0f * PxPi / 180.0f;
	const PxReal rc = kChuteR + 0.5f * kChuteWall;
	for (int k = 0; k < nseg; ++k)
	{
		const PxReal phi = -0.5f * span + (k + 0.5f) * span / nseg;
		const PxVec3 radial = down * cosf(phi) + side * sinf(phi);
		const PxVec3 tangent = axis.cross(radial);
		addPanel(body, mid + radial * rc, axis, tangent, PxVec3(0.5f * len, rc * sinf(0.5f * span / nseg) + 0.002f, 0.5f * kChuteWall));
	}
	gScene->addActor(*body);
}

// Cone / cylinder shell around a vertical axis: radius r0 at the bottom (y = base) to r1 at height h.
static void addCone(const PxVec3& base, PxReal r0, PxReal r1, PxReal h, PxU32 colour)
{
	PxRigidStatic* body = newPiece(PIECE_CHUTE, colour);
	const int nseg = 16;
	const PxReal wallLen = sqrtf(h * h + (r1 - r0) * (r1 - r0));
	for (int k = 0; k < nseg; ++k)
	{
		const PxReal psi = (k + 0.5f) * 2.0f * PxPi / nseg;
		const PxVec3 radial(cosf(psi), 0, sinf(psi)), tangent(-sinf(psi), 0, cosf(psi));
		const PxVec3 wall = (radial * (r1 - r0) + PxVec3(0, h, 0)).getNormalized();
		const PxVec3 centre = base + radial * (0.5f * (r0 + r1)) + PxVec3(0, 0.5f * h, 0);
		addPanel(body, centre, tangent, wall, PxVec3(0.5f * (r0 + r1) * sinf(PxPi / nseg) + 0.002f, 0.5f * wallLen, 0.5f * kChuteWall));
	}
	gScene->addActor(*body);
}

static void buildRun()
{
	const PxVec3 nz(0, 0, 1), up(0, 1, 0), nx(1, 0, 0);
	// Tower backdrop: 4 columns x 5 rows of square tiles standing in the x-y plane behind the track.
	for (int col = 0; col < 4; ++col)
		for (int row = 0; row < 5; ++row)
			addSquareTile(PxVec3((col + 0.5f) * T, (row + 0.5f) * T, -0.5f * kTileT), nz, up, PxU32((col * 2 + row) % 6));

	// Chute levels, top to bottom, alternating direction.  Each drops 0.3 T over its length.
	const PxVec3 L0a(0.00f * T, 4.20f * T, kTrackZ), L0b(3.70f * T, 3.90f * T, kTrackZ);
	const PxVec3 L1a(4.00f * T, 3.10f * T, kTrackZ), L1b(0.30f * T, 2.80f * T, kTrackZ);
	const PxVec3 L2a(0.00f * T, 2.00f * T, kTrackZ), L2b(3.70f * T, 1.70f * T, kTrackZ);
	const PxVec3 L3a(4.00f * T, 0.90f * T, kTrackZ), L3b(0.30f * T, 0.60f * T, kTrackZ);
	addChute(L0a, L0b, 0); addChute(L1a, L1b, 1); addChute(L2a, L2b, 2); addChute(L3a, L3b, 3);

	// Ring tiles the chutes pass through (standing across the track), one per level.
	const PxReal hole = 0.42f * T;
	auto yAt = [](const PxVec3& a, const PxVec3& b, PxReal x) { return a.y + (b.y - a.y) * (x - a.x) / (b.x - a.x); };
	addRingTile(PxVec3(3.4f * T, yAt(L0a, L0b, 3.4f * T) + 0.05f * T, kTrackZ), nx, up, hole, 1);
	addRingTile(PxVec3(0.6f * T, yAt(L1a, L1b, 0.6f * T) + 0.05f * T, kTrackZ), nx, up, hole, 3);
	addRingTile(PxVec3(3.4f * T, yAt(L2a, L2b, 3.4f * T) + 0.05f * T, kTrackZ), nx, up, hole, 5);
	addRingTile(PxVec3(0.6f * T, yAt(L3a, L3b, 0.6f * T) + 0.05f * T, kTrackZ), nx, up, hole, 0);

	// Wall tiles at the drops so a ball flying off a chute end turns down onto the next level.
	addSquareTile(PxVec3(4.05f * T, 3.60f * T, kTrackZ), nx, up, 4);   // right of L0 -> L1
	addSquareTile(PxVec3(-0.05f * T, 2.40f * T, kTrackZ), nx, up, 2);  // left of L1 -> L2
	addSquareTile(PxVec3(4.05f * T, 1.40f * T, kTrackZ), nx, up, 0);   // right of L2 -> L3

	// Funnel on top of the tower, its hole above the start of the top chute; hose sprays into it.
	addCone(PxVec3(0.5f * T, 4.75f * T, kTrackZ), 0.22f * T, 0.65f * T, 0.55f * T, 4);
	// Catch bowl on the floor at the left, where the last chute drops.
	addCone(PxVec3(-0.15f * T, 0.0f, 0.65f * T), 0.85f * T, 0.85f * T, 0.35f * T, 5);
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
	spawnBall(kSpout + PxVec3(0.04f * (gBalls.size() % 3) - 0.04f, 0, 0), gBallRadius);
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
void   setBallRadius(PxReal r)     { gBallRadius = PxClamp(r, 0.012f, 0.032f); }   // must pass the funnel hole
bool   isPaused()                  { return !gIsRunning; }
void   setPaused(bool p)           { gIsRunning = !p; }
void   getRunExtent(PxReal& w, PxReal& h, PxReal& trackZ) { w = kW; h = kH; trackZ = kTrackZ; }

// ---------------------------------------------------------------- hose sprayer + particle budget
// Particles live in a ring: the hose writes new ones at gEmitHead, wrapping at the active cap, so
// once the cap is reached the oldest water (down in the basin) is recycled back to the nozzle.
static const PxVec3 kHoseTip(0.42f * T, 5.75f * T, kTrackZ);           // above the funnel
static const PxVec3 kHoseDir = PxVec3(0.25f, -1.0f, 0.0f).getNormalized();
static const PxReal kHoseSpeed = 1.8f, kHoseRadius = 0.022f;
static const PxU32  kEmitPerFrame = 32;                 // ~1900 particles / s at 60 Hz at full flow
static const PxU32  kEmitMax = 96;                      // array size; flow is clamped so we never exceed it
static PxReal gHoseFlow = 0.2f;                         // 0 .. 1 (the UI shows it as a percentage), scales both rate and speed
static bool  gHoseOn = false;
static PxU32 gMaxActive = 8000, gEmitHead = 0, gActive = 0;

static float frand() { return float(rand()) / float(RAND_MAX); }

static void syncActive()
{
	gParticleBuffer->setNbActiveParticles(gActive);
}

void setMaxActiveParticles(PxU32 cap)
{
	gMaxActive = PxClamp(cap, 100u, gParticleCapacity);
	if (gActive > gMaxActive) gActive = gMaxActive;
	if (gEmitHead >= gMaxActive) gEmitHead = 0;
	if (gParticleBuffer) syncActive();
}
PxU32 getMaxActiveParticles()   { return gMaxActive; }
PxU32 getActiveParticles()      { return gActive; }
PxU32 getParticleCapacity()     { return gParticleCapacity; }
void  setHoseOn(bool on)        { gHoseOn = on; }
void  setHoseFlow(PxReal f)     { gHoseFlow = PxClamp(f, 0.05f, 1.0f); }   // never 0: an "on" hose always sprays
PxReal getHoseFlow()            { return gHoseFlow; }
PxReal getHoseSpeed()           { return kHoseSpeed * gHoseFlow; }
PxU32  getHoseRate()            { return PxU32(kEmitPerFrame * gHoseFlow + 0.5f) * 60u; }
bool  isHoseOn()                { return gHoseOn; }
void  getHose(PxVec3& tip, PxVec3& dir, PxReal& radius) { tip = kHoseTip; dir = kHoseDir; radius = kHoseRadius; }

void clearWater()
{
	gActive = 0; gEmitHead = 0;
	if (gParticleBuffer) syncActive();
}

static void emitFromHose(PxReal dt)
{
	if (!gHoseOn || !gParticleBuffer) return;
	PxCudaContextManager* ccm = gScene->getCudaContextManager();
	PxCudaContext* ctx = ccm->getCudaContext();

	// spray cross-section basis
	const PxVec3 a(0.0f, 0.0f, 1.0f);
	const PxVec3 b = kHoseDir.cross(a).getNormalized();

	const PxU32 n = PxMin(kEmitMax, PxU32(kEmitPerFrame * gHoseFlow + 0.5f));
	const PxReal speed = kHoseSpeed * gHoseFlow;
	if (n == 0) return;
	PxVec4 pos[kEmitMax], vel[kEmitMax];
	PxU32 idx[kEmitMax];
	for (PxU32 i = 0; i < n; ++i)
	{
		const float r = kHoseRadius * sqrtf(frand()), th = 6.2831853f * frand();
		const PxVec3 off = a * (r * cosf(th)) + b * (r * sinf(th));
		const PxVec3 p = kHoseTip + off + kHoseDir * (frand() * speed * dt);
		const PxVec3 v = kHoseDir * (speed * (0.95f + 0.1f * frand())) + off * 2.0f;
		pos[i] = PxVec4(p, gParticleInvMass);
		vel[i] = PxVec4(v, 0.0f);
		idx[i] = gEmitHead;
		gEmitHead = (gEmitHead + 1) % gMaxActive;
		gActive = PxMin(gMaxActive, PxMax(gActive, idx[i] + 1));
	}

	ccm->acquireContext();
	PxVec4* dPos = gParticleBuffer->getPositionInvMasses();
	PxVec4* dVel = gParticleBuffer->getVelocities();
	// indices are consecutive modulo the cap: at most two contiguous runs
	PxU32 start = 0;
	while (start < n)
	{
		PxU32 end = start + 1;
		while (end < n && idx[end] == idx[end - 1] + 1) ++end;
		const size_t bytes = (end - start) * sizeof(PxVec4);
		ctx->memcpyHtoD(CUdeviceptr(dPos + idx[start]), pos + start, bytes);
		ctx->memcpyHtoD(CUdeviceptr(dVel + idx[start]), vel + start, bytes);
		start = end;
	}
	ccm->releaseContext();

	syncActive();
	gParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_POSITION);
	gParticleBuffer->raiseFlags(PxParticleBufferFlag::eUPDATE_VELOCITY);
}

// -----------------------------------------------------------------------------------------------------------------
void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL);

	initScene();
	if (PxCudaContextManager* ccm = gScene->getCudaContextManager())
		printf("PhysX GPU: %s\n", ccm->getDeviceName());
	gMaterial = gPhysics->createMaterial(0.4f, 0.3f, 0.35f);

	// Water: 50k-slot fluid buffer, 12 mm spacing, empty until the hose is switched on.
	initParticles(50000, 0.012f, 1000.0f, 20000);
	gParticleSystem->setMaxVelocity(8.0f);

	// Floor (the deck) and the Magna-Tiles run.
	gScene->addActor(*PxCreatePlane(*gPhysics, PxPlane(0.f, 1.f, 0.f, 0.0f), *gMaterial));
	buildRun();
	dropBall();
	if (getenv("BALLRUN_HOSE")) gHoseOn = true;        // testing aid: start spraying immediately
	printf("Ball run: %u pieces, fluid capacity %u. Keys: H = hose on/off, W = clear water, B = drop ball, M = place mode, X = clear balls, [ ] = ball radius, P = pause, O = step, C = reset view\n"
		"Mouse: left drag = orbit, right drag = pan, wheel / middle drag = zoom\n",
		gNumPieces, gParticleCapacity);
}

// ---------------------------------------------------
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning || gStep)
	{
		gStep = false;
		const PxReal dt = 1.0f / 60.0f;
		++gFrame;
		emitFromHose(dt);
		if (getenv("BALLRUN_TRACE") && gFrame % 30 == 0)
			for (size_t i = 0; i < gBalls.size(); ++i)
			{
				const PxVec3 p = gBalls[i]->getGlobalPose().p, v = gBalls[i]->getLinearVelocity();
				printf("t=%.1f ball%zu pos=(%.3f %.3f %.3f) vel=(%.2f %.2f %.2f) sleeping=%d\n", gFrame / 60.0f, i, p.x, p.y, p.z, v.x, v.y, v.z, int(gBalls[i]->isSleeping()));
			}
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
	case 'H':	gHoseOn = !gHoseOn; break;
	case 'W':	clearWater(); break;
	case '-':	setHoseFlow(gHoseFlow - 0.1f); break;
	case '=':	setHoseFlow(gHoseFlow + 0.1f); break;
	case '[':	setBallRadius(gBallRadius - 0.005f); break;
	case ']':	setBallRadius(gBallRadius + 0.005f); break;
#ifdef RENDER_SNIPPET
	case 'C':	{ extern void resetCamera(); resetCamera(); } break;
	case 'M':	{ extern void togglePlaceMode(); togglePlaceMode(); } break;
	case 'T':	{ extern void toggleBuildMode(); toggleBuildMode(); } break;
	case '1': case '2': case '3': case '4': case '5': case '6':
		{ extern void setTileColour(PxU32 c); setTileColour(PxU32(key - '1')); } break;
	case 'F':	{ extern void toggleWireframe(); toggleWireframe(); } break;
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
