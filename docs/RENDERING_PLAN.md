# Ball run: rendering upgrade plan

Goal: turn the debug view (flat green boxes, blue point-sprite water on grey) into
something that looks like a real Magna-Tile run with water on a table. Simulation stays
PhysX 5.3.1 PBD on the Quadro P500; only the render side changes.

## Constraints that shape the plan

- **GPU**: Quadro P500, 2 GB, ~48 GB/s, no ray tracing. Full OpenGL 4.6 support in the
  580 driver, so modern shader-based GL is fine. Budget the render pass at ~6 ms per frame
  at the current 1882x1016 window; the sim already takes most of the frame.
- **Framework**: the snippet render library is fixed-function GL 2 (immediate mode,
  `gluLookAt`, `glutSolidSphere`). It cannot be bent into PBR. Plan is a new render
  backend file for this snippet only; the shared library stays untouched, and the PhysX
  side (`SnippetPBF.cpp`) keeps its current API (`spawnBall`, `getRunExtent`, ...).
- **Data already on the GPU**: fluid positions and diffuse-particle position/lifetime live
  in CUDA-GL shared VBOs (`SharedGLBuffer`). No readback needed for any pass below.

## Target look

- Magna-Tiles: translucent coloured acrylic (red, blue, yellow, green, purple), bevelled
  edges, a faint magnet dot in each corner. Semi-transparent with a glossy coat.
- Balls: glossy plastic or glass marbles, one colour per ball.
- Container: two glass panes (the z walls) with subtle reflection, a wooden table top as
  the floor, soft neutral backdrop.
- Water: clear, refractive, slightly blue with depth, specular sun highlight, foam and
  spray where it splashes.
- Lighting: one warm key light casting soft shadows plus sky/room ambient from an HDRI.

## Phase 0: modern GL scaffolding (half a day)

1. Request a 3.3 core-compatible context: `glutInitContextVersion(3, 3)` with the
   compatibility profile so the existing overlay code keeps working during the transition.
2. Add `BallRunRenderer.{h,cpp}`: shader loader (vertex/fragment/compute from files in
   `snippets/snippetpbf/shaders/`), FBO helper, fullscreen-triangle helper, camera UBO
   (view, projection, inverse projection, eye position, screen size).
3. Mesh builders: bevelled box for tiles, UV sphere for balls, quads for glass and table.
   Instances come straight from `PxRigidActor` poses each frame.
4. Keep the current overlay and orbit camera; they draw last, on top of the final image.

## Phase 1: PBR surfaces, image-based lighting, shadows (1 to 2 days)

1. **BRDF**: Cook-Torrance with GGX normal distribution, Smith-GGX geometry, Schlick
   Fresnel. Metallic-roughness inputs per material. Linear-space lighting, output through
   an sRGB framebuffer or an explicit conversion in the post pass.
2. **Environment**: one CC0 HDRI (Poly Haven "studio small" or an indoor living-room
   map). At startup convert equirect to a cubemap, then prefilter it: irradiance cubemap
   (diffuse), roughness-mipped specular cubemap, and the split-sum BRDF lookup texture.
   All three are tiny compute or fragment passes run once.
3. **Shadows**: a single 2048x2048 shadow map from the key light with 3x3 PCF. The run is
   small (1.2 m x 1.4 m), so one orthographic map covers it with no cascades.
4. **Translucent tiles**: forward-rendered after opaque geometry, sorted back to front,
   alpha ~0.55 with a full-strength specular term so the coat still shines. Cheap and
   convincing for coloured acrylic. Refraction through tiles is not worth the cost.
5. **Materials table**: tile colours, marble colours, glass (roughness 0.02, IOR 1.5),
   wood table (albedo texture, roughness 0.6). Ball colour assigned at spawn.
6. **Ambient occlusion**: skip SSAO in the first pass; the IBL plus shadow map carries most
   of the depth cue. Revisit if the table looks floaty.

## Phase 2: screen-space fluid rendering (2 to 3 days)

This is the "NVIDIA Flex" water look and is the single biggest visual win. Nothing here
touches the simulation.

1. **Depth pass**: draw the fluid VBO as point sprites; in the fragment shader ray-cast a
   sphere per sprite and write `gl_FragDepth`, so particles become overlapping spheres.
   Half resolution is enough on the P500.
2. **Depth smoothing**: narrow-range filter (Truong and Yuksel 2018) rather than plain
   bilateral or curvature flow. It removes the "bag of balls" look without bleeding
   across silhouettes and needs only 2 to 3 iterations.
3. **Thickness pass**: same sprites, additive blend of a soft disc, into an R16F target.
   Drives colour absorption and how refractive the surface is.
4. **Normals**: reconstruct from the smoothed depth with the inverse projection.
5. **Composite** (fullscreen pass, before the transparent tiles):
   - Refraction: sample the opaque scene colour offset by `normal.xy * thickness * k`.
   - Absorption: Beer-Lambert `exp(-absorb * thickness)` with a blue-green tint.
   - Reflection: Schlick Fresnel blend of the specular environment cubemap.
   - Specular highlight from the key light with a low roughness.
6. **Foam and spray**: PhysX already classifies diffuse particles (spray, foam, bubbles are
   controlled by `PxDiffuseParticleParams`). Render them as soft white sprites whose alpha
   fades with the lifetime stored in the position buffer's w component. Bubbles under the
   surface get a small refraction wobble; keep it simple at first.
7. **Sorting with glass**: the water composite writes depth, so the glass panes and tiles
   blend correctly afterwards.

## Phase 3: post and polish (1 day)

1. HDR render target, ACES tone mapping, then sRGB.
2. FXAA on the final image (MSAA does not play well with the fluid passes).
3. A very light bloom on the specular highlights only.
4. Soft particle edges where water meets tiles (fade by depth difference).
5. Camera: keep the orbit, add a smooth "follow the newest ball" toggle for the kid.

## Simulation-side tweaks that make the water read better

Cheap and independent of the renderer; worth doing first.

- Spacing 12 mm to 8 mm raises the count from ~6.8k to ~23k particles. The P500 handles
  that at interactive rates and the surface smooths far better.
- `PxPBDMaterial`: raise viscosity slightly, set `surfaceTension` and `cohesion` so drops
  bead instead of scattering, and `adhesion` low so water sheets off the tiles.
- `PxDiffuseParticleParams`: enable buoyancy for bubbles, shorter `lifetime`, and raise
  `threshold` so foam appears only at real splashes rather than everywhere.
- 4 to 6 substeps per 60 Hz frame so thin sheets on steep ramps do not tunnel.

## Alternative for cinematic output (optional)

For a "movie" of the run rather than a live view, dump per-frame fluid positions, diffuse
particles, and ball poses to `.npz` and import into Blender: particles to a volume via
Geometry Nodes (points to volume, volume to mesh), then Cycles with a glass water shader.
Renders at any quality the laptop can wait for. Not interactive and not needed for play,
but it produces the best-looking result by a wide margin.

## Order of work and expected payoff

| Step | Effort | Visual payoff |
|---|---|---|
| Sim tweaks (spacing, material) | hours | medium |
| Phase 0 scaffolding | half day | none, unblocks everything |
| Phase 1 PBR + IBL + shadows | 1 to 2 days | high |
| Phase 2 screen-space water | 2 to 3 days | very high |
| Phase 3 post | 1 day | medium |
