# physx-toddler

A toddler-friendly Magna-Tile ball run with water, built on NVIDIA PhysX 5.3.1
(GPU position-based fluid). It replaces the stock `SnippetPBF` snippet in the
PhysX SDK: balls roll down a set of tilted tiles inside a thin glass channel
while ~6000 fluid particles pour over the same ramps.

## What is in here

| Path | Purpose |
| --- | --- |
| `src/SnippetPBF.cpp` | Scene: 1.2 m x 1.4 m run, tiles, container walls, ball spawning, fluid setup, keyboard handling |
| `src/SnippetPBFRender.cpp` | Rendering: orbit camera, on-screen button panel, click-to-place balls, fluid rendering |
| `patches/physx-5.3.1-linux-build.patch` | Changes to the PhysX Linux build (gcc preset, no `-Werror`, explicit GL/GLU/glut paths) |
| `patches/snippetpbf-vs-upstream.patch` | Diff of `src/` against the stock snippet, for reference |
| `scripts/install.sh` | Copy `src/` into a PhysX checkout and apply the build patch |
| `scripts/build.sh` | Generate the Linux project and build the snippet |
| `scripts/sync-back.sh` | Copy edits made in the PhysX tree back into this repo |

The code lives inside the PhysX snippet framework, so this repo is not a
standalone build. It needs a PhysX 5.3.1 checkout
(https://github.com/NVIDIA-Omniverse/PhysX, tag 105.1-physx-5.3.1) and a CUDA-capable GPU.

## Build

```sh
scripts/install.sh ~/src/PhysX-5.3.1
scripts/build.sh   ~/src/PhysX-5.3.1
~/src/PhysX-5.3.1/physx/bin/linux.clang/release/SnippetPBF_64
```

The build patch hard-codes GL, GLU and glut from a micromamba `cuda12`
environment under `~/.local/share/micromamba/envs/cuda12/lib`. Edit the three
`Snippet*.cmake` hunks in the patch if your libraries live elsewhere.

## Controls

Keyboard:

| Key | Action |
| --- | --- |
| `B` | Drop a ball from the spout |
| `M` | Toggle place mode (ghost ball follows the cursor, click to drop it) |
| `X` | Clear all balls |
| `[` / `]` | Shrink / grow the ball radius (15 mm to 80 mm) |
| `P` | Pause / resume |
| `O` | Single step while paused |
| `C` | Reset the camera |

Mouse: left drag orbits, right drag pans, wheel or middle drag zooms. The
button panel in the top-left corner exposes the same actions for a child who
cannot use the keyboard yet.

## Scene notes

- Units are metres. Tiles are 12 mm thick boxes; the channel is 0.24 m deep.
- Up to 30 balls, spawned as light plastic (about 54 g at the default 35 mm radius).
- Fluid is a 21 x 17 x 19 block of particles at 12 mm spacing, density 1000, max velocity 8 m/s.
- PhysX Visual Debugger (PVD) is disabled; the GPU device name is printed at startup.

## Workflow

Edit either here and run `scripts/install.sh`, or edit inside the PhysX tree
and run `scripts/sync-back.sh` to bring changes back before committing.
