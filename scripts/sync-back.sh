#!/usr/bin/env bash
# Pull edits made inside the PhysX tree back into this repo.
# Usage: scripts/sync-back.sh [PHYSX_DIR]   (default: ~/src/PhysX-5.3.1)
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
PHYSX=${1:-$HOME/src/PhysX-5.3.1}
SRC=$PHYSX/physx/snippets/snippetpbf
cp "$SRC"/SnippetPBF.cpp "$SRC"/SnippetPBFRender.cpp "$HERE"/src/
git -C "$PHYSX" diff -- physx/buildtools physx/snippets/compiler physx/source/compiler > "$HERE/patches/physx-5.3.1-linux-build.patch"
git -C "$PHYSX" diff -- physx/snippets/snippetpbf > "$HERE/patches/snippetpbf-vs-upstream.patch"
git -C "$HERE" status --short
