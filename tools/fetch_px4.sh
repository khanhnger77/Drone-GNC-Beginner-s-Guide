#!/bin/bash
# fetch_px4.sh
#
# Author: Khanh Ngo
#
# Clone the pinned PX4 release next to this repository's code, for reading PX4
# side by side with the lessons. Source only: no submodules, so it cannot build SITL.
# To build SITL, run "git submodule update --init --recursive" inside the clone.

set -e

PX4_TAG="v1.17.0"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET="$ROOT/external/PX4-Autopilot"

if [ -d "$TARGET/.git" ]; then
    echo "PX4 already at $TARGET ($(git -C "$TARGET" describe --tags))"
    exit 0
fi

git clone --depth 1 --branch "$PX4_TAG" https://github.com/PX4/PX4-Autopilot.git "$TARGET"
echo "PX4 $PX4_TAG cloned to $TARGET"
