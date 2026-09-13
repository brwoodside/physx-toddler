# physx-toddler

A toddler-friendly Magna-Tiles ball run with water, built on NVIDIA PhysX 5.3.1
(GPU position-based fluid). It replaces the stock `SnippetPBF` snippet in the
PhysX SDK. The scene is modelled on the Lakeshore Magna-Tiles ball run set: a
tower of coloured square tiles, translucent half-pipe chutes zig-zagging down
through ring tiles, a funnel on top, and a catch bowl at the bottom. A garden
hose sprayer pours water into the funnel and balls roll the same course.

## What is in here

| Path | Purpose |
| --- | --- |
| `src/SnippetPBF.cpp` | Physics: Magna-Tiles piece builders and layout, hose emitter, particle budget, ball and tile editor APIs, keyboard handling |
| `src/SnippetPBFRender.cpp` | Rendering and UI: orbit camera, procedural sky and wooden deck, sphere-impostor water, sprayer model, paramgl-style panel, place and build modes |
| `docs/PIECES.md` | Catalogue of the Magna-Tiles set pieces derived from the product photo |
| `docs/RENDERING_PLAN.md` | Plan for physically based rendering and screen-space fluid |
| `patches/physx-5.3.1-linux-build.patch` | Changes to the PhysX Linux build (gcc preset, no `-Werror`, explicit GL/GLU/glut paths) |
| `patches/snippetpbf-vs-upstream.patch` | Diff of `src/` against the stock snippet, for reference |
| `scripts/install.sh` | Copy `src/` into a PhysX checkout and apply the build patch |
| `scripts/build.sh` | Generate the Linux project and build the snippet |
| `scripts/sync-back.sh` | Copy edits made in the PhysX tree back into this repo |

The code lives inside the PhysX snippet framework, so this repo is not a
standalone build. It needs a PhysX 5.3.1 checkout
(https://github.com/NVIDIA-Omniverse/PhysX, tag 105.1-physx-5.3.1) and a CUDA-capable GPU.
PhysX 5.3.1 is the last release whose prebuilt GPU library still runs on Pascal
cards; newer releases drop it.

## Build

```sh
scripts/install.sh ~/src/PhysX-5.3.1
scripts/build.sh   ~/src/PhysX-5.3.1
~/src/PhysX-5.3.1/physx/bin/linux.clang/release/SnippetPBF_64
```

The build patch hard-codes GL, GLU and glut from a micromamba `cuda12`
environment under `~/.local/share/micromamba/envs/cuda12/lib`. Edit the three
`Snippet*.cmake` hunks in the patch if your libraries live elsewhere.

On a hybrid-graphics laptop run the binary through a PRIME offload wrapper so
the OpenGL window and the CUDA simulation share the NVIDIA GPU.

## Controls

The panel in the top-left corner exposes everything below as clickable rows,
sliders and switches, for a child who cannot use the keyboard yet.

| Key | Action |
| --- | --- |
| `H` | Hose on / off (or click the sprayer nozzle) |
| `-` / `=` | Hose flow down / up (5% to 100%) |
| `W` | Clear all water |
| `B` | Drop a ball into the funnel |
| `M` | Place mode: a ghost ball follows the cursor, click to drop it |
| `X` | Clear all balls |
| `[` / `]` | Shrink / grow the ball radius (12 mm to 32 mm) |
| `T` | Build mode: click to add a tile, drag a tile to move it, right-click to remove |
| `1` to `6` | Tile colour for build mode |
| `P` | Pause / resume |
| `O` | Single step while paused |
| `F` | Wireframe outlines on / off |
| `C` | Reset the camera |
| `Esc` | Quit |

Mouse: left drag orbits, right drag pans, wheel or middle drag zooms.

Sliders: max particles (100 to 50,000, log scale), hose flow, ball radius,
UI size.

## Scene notes

- Units are metres. The set is built at twice real size: one tile is 150 mm,
  so the 12 mm water particles resolve a chute about nine particles wide.
- Pieces are static actors made of box panels. Ring tiles have an octagonal
  hole, chutes are seven-segment half-pipes, the funnel and bowl are
  sixteen-segment cones. Each actor's `userData` tags its kind and colour.
- Tile actors are posed at their centre so the build-mode editor can move
  them with a single pose update. Dragging keeps a tile in its own plane.
- Water lives in a 50,000-slot buffer that starts empty. The hose writes new
  particles straight into GPU memory as a ring, so once the max-particles cap
  is reached the oldest water is recycled back to the nozzle.
- Up to 30 balls, cycling the six Magna-Tiles colours.
- Environment variables for testing: `BALLRUN_HOSE=1` starts the hose
  spraying; `BALLRUN_TRACE=1` logs ball positions and editor events.

## Workflow

Edit either here and run `scripts/install.sh`, or edit inside the PhysX tree
and run `scripts/sync-back.sh` to bring changes back before committing.
