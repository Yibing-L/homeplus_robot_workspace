#!/usr/bin/env bash
# Native nvblox for ROS 2 Humble; no Isaac ROS/NITROS or PyTorch dependency.
set -euo pipefail
NVBLOX_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NVBLOX_DEPS="$NVBLOX_WORKSPACE/.deps"
NVBLOX_SOURCE_DIR="${NVBLOX_SOURCE_DIR:-$NVBLOX_DEPS/src/nvblox}"
NVBLOX_BUILD_DIR="${NVBLOX_BUILD_DIR:-$NVBLOX_DEPS/build/nvblox}"
NVBLOX_REVISION=c457c3fc01003bec6eba3ec1c61e6bf84bc3f51f # v0.0.10
mkdir -p "$NVBLOX_DEPS"
touch "$NVBLOX_DEPS/COLCON_IGNORE"
if [[ ! -d "$NVBLOX_SOURCE_DIR/.git" ]]; then
  git clone --depth 1 --branch v0.0.10 https://github.com/nvidia-isaac/nvblox.git "$NVBLOX_SOURCE_DIR"
fi
[[ "$(git -C "$NVBLOX_SOURCE_DIR" rev-parse HEAD)" == "$NVBLOX_REVISION" ]] || {
  echo "Unexpected nvblox source revision; expected $NVBLOX_REVISION" >&2; exit 1;
}
cmake -S "$NVBLOX_SOURCE_DIR" -B "$NVBLOX_BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES="${NVBLOX_CUDA_ARCHITECTURES:-89}" \
  -DBUILD_TESTING=OFF -DBUILD_BENCHMARKS=OFF \
  -DREGISTER_INSTALL_PREFIX=OFF -DREGISTER_BUILD_DIR=OFF \
  -DBUILD_PYTORCH_WRAPPER=OFF -DBUILD_RENDERER=OFF \
  -DCMAKE_INSTALL_PREFIX="$NVBLOX_DEPS/nvblox"
cmake --build "$NVBLOX_BUILD_DIR" --parallel "${NVBLOX_BUILD_JOBS:-4}"
cmake --install "$NVBLOX_BUILD_DIR"
echo "Installed nvblox v0.0.10 in $NVBLOX_DEPS/nvblox"
