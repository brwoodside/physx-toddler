#!/usr/bin/env bash
# Generate the Linux release project (first time only) and build SnippetPBF.
# Usage: scripts/build.sh [PHYSX_DIR]   (default: ~/src/PhysX-5.3.1)
set -euo pipefail
PHYSX=${1:-$HOME/src/PhysX-5.3.1}
cd "$PHYSX/physx"
[ -f compiler/linux-release/Makefile ] || ./generate_projects.sh linux
make -C compiler/linux-release -j"$(nproc)" SnippetPBF
echo "binary: $PHYSX/physx/bin/linux.clang/release/SnippetPBF_64"
