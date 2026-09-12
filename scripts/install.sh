#!/usr/bin/env bash
# Copy the ball run sources into a PhysX 5.3.1 checkout and apply the Linux build patch.
# Usage: scripts/install.sh [PHYSX_DIR]   (default: ~/src/PhysX-5.3.1)
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
PHYSX=${1:-$HOME/src/PhysX-5.3.1}
DEST=$PHYSX/physx/snippets/snippetpbf
[ -d "$DEST" ] || { echo "not a PhysX checkout: $PHYSX" >&2; exit 1; }
cp "$HERE"/src/SnippetPBF.cpp "$HERE"/src/SnippetPBFRender.cpp "$DEST"/
if git -C "$PHYSX" apply --check "$HERE/patches/physx-5.3.1-linux-build.patch" 2>/dev/null; then
  git -C "$PHYSX" apply "$HERE/patches/physx-5.3.1-linux-build.patch"
  echo "applied build patch"
else
  echo "build patch already applied or does not apply cleanly; skipping"
fi
echo "sources installed into $DEST"
