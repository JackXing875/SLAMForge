#!/bin/bash
set -e
cd /home/schrieffer/LiteVO/build
cmake .. -DLITEVO_BUILD_TESTS=ON -DLITEVO_ENABLE_CERES=ON -DLITEVO_ENABLE_G2O=OFF
make -j$(nproc) 2>&1
echo "BUILD COMPLETED"
