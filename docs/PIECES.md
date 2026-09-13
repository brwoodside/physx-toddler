# Magna-Tiles ball run: piece catalogue

Derived from the Lakeshore product photo (`~/Downloads/lakeshore-ballrun.webp`). Real
Magna-Tiles squares are 3 in (76 mm) on a side; everything below is in that unit ("1 tile").

## Structure (translucent coloured acrylic, magnets in the edges)

| Piece | Size | Colours | Notes |
|---|---|---|---|
| Square tile | 1 x 1 | red, blue, yellow, green, purple, orange | Solid face, bevelled frame. Walls and towers are stacks of these. |
| Ring tile | 1 x 1 with a round hole | same six colours | A chute passes through the hole. Used as track supports and as the "window" the ball drops through. |
| Right triangle | half a square | same | Seen only sparingly in the photo. Optional. |

## Track (smoky translucent blue plastic, snaps to tile edges)

| Piece | Footprint | Notes |
|---|---|---|
| Straight chute | 1 tile long, half-pipe cross-section | The workhorse. Chains through ring tiles. Slight downhill when tiles are stepped. |
| Curved chute | quarter circle, 1 tile radius | Turns the track 90 degrees. Two make a U-turn. |
| Drop bowl / funnel | 1 tile wide, sits on top of a tile stack | The entry at the top of the tower. Ball spirals then drops through the hole. |
| Catch bowl | 1 tile wide dish | The end piece at the bottom. Also a mid-run spiral bowl in the photo. |
| Spiral / half-turn bowl | ~1.5 tile wide | The wide dish mid-tower that swings the ball around a corner. |
| Exit ramp | 1 tile, open end | Short straight that leaves the last tile and drops the ball into the catch bowl. |

## Balls

Glass-look spheres, about 25 mm diameter, in red, blue, yellow, green, orange, purple.
No black or white. The simulator now cycles exactly this palette.

## Water (our addition)

The hose is not a Magna-Tiles part. It replaces the ball drop at the top funnel.

## What this means for the simulator

The current run is made of long straight "planks" that do not exist in the set. The next
build step is to replace them with the catalogue above:

1. Square and ring tiles as thin bevelled boxes (ring tiles as a box with a cylinder cut,
   approximated by four boxes around the hole for physics).
2. Straight chute as a half-pipe: physics from 6 to 8 thin box segments around the arc,
   rendering from a smooth extruded mesh. Curved chute the same with the arc swept.
3. Funnel and bowls as revolved profiles: a PhysX triangle mesh (static) for collision,
   which the PBD particles and balls both support.
4. A layout file listing pieces by type, grid position, rotation and colour, so runs can be
   designed without a rebuild. The photo's tower is roughly a 3-wide, 5-high stack with the
   track corkscrewing down through ring tiles, then a ground-level S-curve into the catch bowl.
