#!/bin/bash
set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

build_variant() {
  local variant="$1"
  local build_name="${2:-${variant}}"
  local build_dir="${ROOT_DIR}/build-${build_name}"

  cmake -S "${ROOT_DIR}" -B "${build_dir}" \
    -Wno-dev \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_PREFIX_PATH="/usr/local;/usr/local/opt/yaml-cpp;/usr/local/opt/google-sparsehash" \
    -DCMAKE_CXX_FLAGS="-I/usr/local/include -I/usr/local/opt/yaml-cpp/include -I/usr/local/opt/google-sparsehash/include" \
    -DLAZYCBS_BUILD_VARIANT="${variant}"

  cmake --build "${build_dir}" -- -j1
}

build_variant og
build_variant target
build_variant confilct
