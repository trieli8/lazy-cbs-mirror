#!/bin/bash
set -e
rm -rf build
mkdir build
cd build
cmake .. \
  -Wno-dev \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_PREFIX_PATH="/usr/local;/usr/local/opt/yaml-cpp;/usr/local/opt/google-sparsehash" \
  -DCMAKE_CXX_FLAGS="-I/usr/local/include -I/usr/local/opt/yaml-cpp/include -I/usr/local/opt/google-sparsehash/include"
make -j1
