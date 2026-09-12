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

#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "../snippetrender/SnippetRender.h"
#include "../snippetrender/SnippetCamera.h"
#include "../snippetrender/SnippetFontRenderer.h"
#include <stdio.h>

#define CUDA_SUCCESS 0
#define SHOW_SOLID_SDF_SLICE 0
#define IDX(i, j, k, offset) ((i) + dimX * ((j) + dimY * ((k) + dimZ * (offset))))
using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxPBDParticleSystem* getParticleSystem();
extern PxParticleAndDiffuseBuffer* getParticleBuffer();

extern int getNumDiffuseParticles();
extern PxRigidDynamic* spawnBall(const PxVec3& pos, PxReal r);
extern void   dropBall();
extern void   clearBalls();
extern PxU32  getNumBalls();
extern PxU32  getMaxBalls();
extern PxReal getBallRadius();
extern void   setBallRadius(PxReal r);
extern bool   isPaused();
extern void   setPaused(bool p);
extern void   getRunExtent(PxReal& w, PxReal& h, PxReal& halfDepth);


namespace
{
Snippets::Camera* sCamera;

Snippets::SharedGLBuffer sPosBuffer;
Snippets::SharedGLBuffer sDiffusePosLifeBuffer;

// ---------------------------------------------------------------- orbit camera
// Left drag: orbit around the run.  Right drag: pan.  Wheel / middle drag: zoom.  C: reset view.
static const PxVec3 kHomeTarget(0.6f, 0.7f, 0.0f);
static const float  kHomeDist = 2.4f;
PxVec3 sTarget = kHomeTarget;
float  sYaw = 0.0f, sPitch = 0.0f, sDist = kHomeDist;
int    sLastX = 0, sLastY = 0, sButton = -1;

void applyOrbit()
{
	const float cp = cosf(sPitch), sp = sinf(sPitch);
	const PxVec3 dir(-sinf(sYaw) * cp, -sp, -cosf(sYaw) * cp);   // eye -> target
	sCamera->setPose(sTarget - dir * sDist, dir);
}

PxVec3 orbitDir()
{
	const float cp = cosf(sPitch), sp = sinf(sPitch);
	return PxVec3(-sinf(sYaw) * cp, -sp, -cosf(sYaw) * cp);
}

// ---------------------------------------------------------------- UI overlay + ball placement
// Panel of clickable buttons (top-left).  "Place mode": a ghost ball follows the cursor on the
// run's mid-plane (z = 0); a click (not a drag) drops a ball there.  Drags still orbit / pan / zoom.
enum ButtonId { BTN_DROP, BTN_PLACE, BTN_RAD_MINUS, BTN_RAD_PLUS, BTN_CLEAR, BTN_PAUSE, BTN_VIEW, BTN_COUNT };
struct Button { int x, y, w, h; ButtonId id; };   // pixels, origin top-left
static const int kPanelX = 12, kPanelY = 12, kPanelW = 260, kBtnH = 30, kGap = 6;
Button sButtons[BTN_COUNT];
int    sHoverBtn = -1;
bool   sPlaceMode = false;
int    sCursorX = -1, sCursorY = -1;
int    sPressX = 0, sPressY = 0;
bool   sGhostValid = false;
PxVec3 sGhostPos(0.0f);

int layoutButtons()   // returns panel height in pixels
{
	int y = kPanelY + 8 + 3 * 18 + 6;   // below three status lines
	const int x = kPanelX + 8, w = kPanelW - 16, hw = (w - kGap) / 2;
	int n = 0;
	sButtons[n++] = Button{ x, y, w, kBtnH, BTN_DROP };           y += kBtnH + kGap;
	sButtons[n++] = Button{ x, y, w, kBtnH, BTN_PLACE };          y += kBtnH + kGap;
	sButtons[n++] = Button{ x, y, hw, kBtnH, BTN_RAD_MINUS };
	sButtons[n++] = Button{ x + hw + kGap, y, hw, kBtnH, BTN_RAD_PLUS };  y += kBtnH + kGap;
	sButtons[n++] = Button{ x, y, w, kBtnH, BTN_CLEAR };          y += kBtnH + kGap;
	sButtons[n++] = Button{ x, y, hw, kBtnH, BTN_PAUSE };
	sButtons[n++] = Button{ x + hw + kGap, y, hw, kBtnH, BTN_VIEW };      y += kBtnH + kGap;
	return y + 8 - kPanelY;
}

int hitButton(int x, int y)
{
	for (int i = 0; i < BTN_COUNT; ++i)
	{
		const Button& b = sButtons[i];
		if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return i;
	}
	return -1;
}

bool inPanel(int x, int y)
{
	const int ph = layoutButtons();
	return x >= kPanelX && x < kPanelX + kPanelW && y >= kPanelY && y < kPanelY + ph;
}

void pressButton(ButtonId id)
{
	switch (id)
	{
	case BTN_DROP:      dropBall(); break;
	case BTN_PLACE:     sPlaceMode = !sPlaceMode; break;
	case BTN_RAD_MINUS: setBallRadius(getBallRadius() - 0.005f); break;
	case BTN_RAD_PLUS:  setBallRadius(getBallRadius() + 0.005f); break;
	case BTN_CLEAR:     clearBalls(); break;
	case BTN_PAUSE:     setPaused(!isPaused()); break;
	case BTN_VIEW:      sTarget = kHomeTarget; sYaw = 0.0f; sPitch = 0.0f; sDist = kHomeDist; break;
	default: break;
	}
}

// Cursor -> world ray, using the same basis gluLookAt builds in startRender (up = +y).
PxVec3 cursorRay(int x, int y)
{
	const float w = float(Snippets::getScreenWidth()), h = float(Snippets::getScreenHeight());
	const float u = (float(x) / w) * 2.0f - 1.0f;
	const float v = 1.0f - (float(y) / h) * 2.0f;
	const float t = tanf(PxDegToRad(30.0f));   // half of the 60 degree vertical FOV
	const PxVec3 f = orbitDir();
	const PxVec3 sdir = f.cross(PxVec3(0, 1, 0)).getNormalized();
	const PxVec3 up = sdir.cross(f);
	return (f + sdir * (u * t * w / h) + up * (v * t)).getNormalized();
}

void updateGhost()
{
	sGhostValid = false;
	if (!sPlaceMode || sCursorX < 0 || inPanel(sCursorX, sCursorY)) return;
	const PxVec3 eye = sTarget - orbitDir() * sDist;
	const PxVec3 ray = cursorRay(sCursorX, sCursorY);
	if (fabsf(ray.z) < 1e-4f) return;
	const float t = -eye.z / ray.z;          // hit the z = 0 mid-plane
	if (t <= 0.0f) return;
	PxReal w, h, d; getRunExtent(w, h, d);
	const PxReal r = getBallRadius();
	PxVec3 p = eye + ray * t;
	p.x = PxClamp(p.x, r + 0.005f, w - r - 0.005f);
	p.y = PxClamp(p.y, r + 0.005f, h + 0.3f);
	p.z = 0.0f;
	sGhostPos = p;
	sGhostValid = true;
}

void orbitMouse(int button, int state, int x, int y)
{
	sCursorX = x; sCursorY = y;
	if (button == 3 || button == 4)   // freeglut wheel up / down
	{
		if (state == GLUT_DOWN && !inPanel(x, y))
			sDist = PxClamp(sDist * (button == 3 ? 0.9f : 1.1f), 0.2f, 20.0f);
		return;
	}
	if (state == GLUT_DOWN)
	{
		sPressX = x; sPressY = y;
		if (button == GLUT_LEFT_BUTTON && inPanel(x, y))
		{
			const int b = hitButton(x, y);
			if (b >= 0) pressButton(sButtons[b].id);
			sButton = -1;
			return;
		}
		sButton = button;
	}
	else
	{
		const bool click = abs(x - sPressX) < 4 && abs(y - sPressY) < 4;
		if (button == GLUT_LEFT_BUTTON && click && sPlaceMode && !inPanel(x, y))
		{
			updateGhost();
			if (sGhostValid) spawnBall(sGhostPos, getBallRadius());
		}
		sButton = -1;
	}
	sLastX = x; sLastY = y;
}

void orbitPassiveMotion(int x, int y)
{
	sCursorX = x; sCursorY = y;
	sHoverBtn = hitButton(x, y);
}

void orbitMotion(int x, int y)
{
	sCursorX = x; sCursorY = y;
	const int dx = x - sLastX, dy = y - sLastY;
	sLastX = x; sLastY = y;
	if (sButton == GLUT_LEFT_BUTTON)
	{
		sYaw   -= float(dx) * 0.005f;
		sPitch  = PxClamp(sPitch + float(dy) * 0.005f, -1.55f, 1.55f);
	}
	else if (sButton == GLUT_RIGHT_BUTTON)
	{
		const PxVec3 dir = orbitDir();
		const PxVec3 right = dir.cross(PxVec3(0, 1, 0)).getNormalized();
		const PxVec3 up = right.cross(dir);
		const float k = sDist * 0.0015f;
		sTarget += right * (-float(dx) * k) + up * (float(dy) * k);
	}
	else if (sButton == GLUT_MIDDLE_BUTTON)
	{
		sDist = PxClamp(sDist * expf(float(dy) * 0.01f), 0.2f, 20.0f);
	}
}

void drawGhostBall()
{
	updateGhost();
	if (!sGhostValid) return;
	glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPushMatrix();
	glTranslatef(sGhostPos.x, sGhostPos.y, sGhostPos.z);
	glColor4f(0.3f, 1.0f, 0.4f, 0.35f);
	glutSolidSphere(getBallRadius(), 20, 14);
	glColor4f(0.3f, 1.0f, 0.4f, 0.9f);
	glLineWidth(1.5f);
	glutWireSphere(getBallRadius() * 1.01f, 12, 8);
	glPopMatrix();
	glPopAttrib();
}

void rect(int x, int y, int w, int h, float r, float g, float b, float a)   // pixel coords, origin top-left
{
	const float H = float(Snippets::getScreenHeight());
	glColor4f(r, g, b, a);
	glBegin(GL_QUADS);
	glVertex2f(float(x), H - float(y));
	glVertex2f(float(x + w), H - float(y));
	glVertex2f(float(x + w), H - float(y + h));
	glVertex2f(float(x), H - float(y + h));
	glEnd();
}

void text(int x, int yTop, int px, const char* str, float r = 1.0f, float g = 1.0f, float b = 1.0f)
{
	const float W = float(Snippets::getScreenWidth()), H = float(Snippets::getScreenHeight());
	GLFontRenderer::setColor(r, g, b, 1.0f);
	GLFontRenderer::print(float(x) / W, (H - float(yTop + px)) / H, float(px) / H, str);
}

void drawOverlay()
{
	const int W = int(Snippets::getScreenWidth()), H = int(Snippets::getScreenHeight());
	const int panelH = layoutButtons();

	glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
	glOrtho(0, W, 0, H, -1, 1);
	glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();

	rect(kPanelX, kPanelY, kPanelW, panelH, 0.05f, 0.05f, 0.08f, 0.72f);

	char label[64];
	for (int i = 0; i < BTN_COUNT; ++i)
	{
		const Button& b = sButtons[i];
		const bool hot = (i == sHoverBtn);
		const bool on = (b.id == BTN_PLACE && sPlaceMode) || (b.id == BTN_PAUSE && isPaused());
		if (on)       rect(b.x, b.y, b.w, b.h, 0.15f, 0.55f, 0.30f, hot ? 1.0f : 0.9f);
		else if (hot) rect(b.x, b.y, b.w, b.h, 0.35f, 0.45f, 0.70f, 1.0f);
		else          rect(b.x, b.y, b.w, b.h, 0.22f, 0.26f, 0.40f, 0.95f);
		switch (b.id)
		{
		case BTN_DROP:      strcpy(label, "Drop ball at spout  (B)"); break;
		case BTN_PLACE:     strcpy(label, sPlaceMode ? "Place mode: ON  (M)" : "Place mode: OFF  (M)"); break;
		case BTN_RAD_MINUS: strcpy(label, "Smaller  ["); break;
		case BTN_RAD_PLUS:  strcpy(label, "Bigger  ]"); break;
		case BTN_CLEAR:     strcpy(label, "Clear balls  (X)"); break;
		case BTN_PAUSE:     strcpy(label, isPaused() ? "Resume  (P)" : "Pause  (P)"); break;
		case BTN_VIEW:      strcpy(label, "Reset view  (C)"); break;
		default: label[0] = 0;
		}
		text(b.x + 8, b.y + 7, 15, label);
	}

	text(kPanelX + 8, kPanelY + 6, 15, "BALL RUN", 0.85f, 0.9f, 1.0f);
	snprintf(label, sizeof(label), "Balls %u / %u    radius %.0f mm", getNumBalls(), getMaxBalls(), getBallRadius() * 1000.0f);
	text(kPanelX + 8, kPanelY + 6 + 18, 14, label, 0.8f, 0.8f, 0.85f);
	text(kPanelX + 8, kPanelY + 6 + 36, 13, sPlaceMode ? "Click in the run to place a ball" : "Drag: orbit   R-drag: pan   Wheel: zoom", 0.7f, 0.9f, 0.7f);

	glMatrixMode(GL_PROJECTION); glPopMatrix();
	glMatrixMode(GL_MODELVIEW); glPopMatrix();
	glPopAttrib();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
}

void onBeforeRenderParticles()
{	
}

void renderParticles()
{

	PxPBDParticleSystem* particleSystem = getParticleSystem();
	if (particleSystem)
	{
		PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();
		PxVec4* positions = userBuffer->getPositionInvMasses();
		PxVec4* diffusePositions = userBuffer->getDiffusePositionLifeTime();

		const PxU32 numParticles = userBuffer->getNbActiveParticles();
		const PxU32 numDiffuseParticles = userBuffer->getNbActiveDiffuseParticles();

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();

		cudaContextManager->acquireContext();

		PxCudaContext* cudaContext = cudaContextManager->getCudaContext();
		cudaContext->memcpyDtoH(sPosBuffer.map(), CUdeviceptr(positions), sizeof(PxVec4) * numParticles);
		cudaContext->memcpyDtoH(sDiffusePosLifeBuffer.map(), CUdeviceptr(diffusePositions), sizeof(PxVec4) * numDiffuseParticles);

		cudaContextManager->releaseContext();

#if SHOW_SOLID_SDF_SLICE
		particleSystem->copySparseGridData(sSparseGridSolidSDFBufferD, PxSparseGridDataFlag::eGRIDCELL_SOLID_GRADIENT_AND_SDF);
#endif
	}

	sPosBuffer.unmap();
	sDiffusePosLifeBuffer.unmap();
	PxVec3 color(0.5f, 0.5f, 1);
	Snippets::DrawPoints(sPosBuffer.vbo, sPosBuffer.size / sizeof(PxVec4), color, 4.f);

	PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();

	const PxU32 numActiveDiffuseParticles = userBuffer->getNbActiveDiffuseParticles();

	//printf("NumActiveDiffuse = %i\n", numActiveDiffuseParticles);

	if (numActiveDiffuseParticles > 0)
	{
		PxVec3 colorDiffuseParticles(1, 1, 1);
		Snippets::DrawPoints(sDiffusePosLifeBuffer.vbo, numActiveDiffuseParticles, colorDiffuseParticles, 2.f);
	}
	
	Snippets::DrawFrame(PxVec3(0, 0, 0));
}

void allocParticleBuffers()
{
	PxScene* scene;
	PxGetPhysics().getScenes(&scene, 1);
	PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();

	PxParticleAndDiffuseBuffer* userBuffer = getParticleBuffer();

	const PxU32 maxParticles = userBuffer->getMaxParticles();
	const PxU32 maxDiffuseParticles = userBuffer->getMaxDiffuseParticles();

	sDiffusePosLifeBuffer.initialize(cudaContextManager);
	sDiffusePosLifeBuffer.allocate(maxDiffuseParticles * sizeof(PxVec4));
	
	sPosBuffer.initialize(cudaContextManager);
	sPosBuffer.allocate(maxParticles * sizeof(PxVec4));
}

void clearupParticleBuffers()
{
	sPosBuffer.release();
	sDiffusePosLifeBuffer.release();
}

void renderCallback()
{
	applyOrbit();
	onBeforeRenderParticles();

	stepPhysics(true);

	Snippets::startRender(sCamera);

	PxScene* scene;
	PxGetPhysics().getScenes(&scene,1);
	PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
		Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
	}
	
	renderParticles();
	drawGhostBall();
	drawOverlay();

	Snippets::showFPS();

	Snippets::finishRender();
}

void cleanup()
{
	delete sCamera;
	clearupParticleBuffers();
	cleanupPhysics(true);
}

void exitCallback(void)
{
}
}

void togglePlaceMode()
{
	sPlaceMode = !sPlaceMode;
}

void resetCamera()
{
	sTarget = kHomeTarget; sYaw = 0.0f; sPitch = 0.0f; sDist = kHomeDist;
}

void renderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(0.6f, 0.72f, 2.4f), PxVec3(0.0f, 0.0f, -1.0f));

	Snippets::setupDefault("Ball run (PhysX 5.3 PBD fluid)", sCamera, keyPress, renderCallback, exitCallback);
	glutMouseFunc(orbitMouse);
	glutMotionFunc(orbitMotion);
	glutPassiveMotionFunc(orbitPassiveMotion);
	applyOrbit();

	initPhysics(true);
	Snippets::initFPS();

	allocParticleBuffers();

	glutMainLoop();

	cleanup();
}
#endif
