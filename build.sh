#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake -S "${project_dir}" -B "${project_dir}/build" \
    -DSLAMFORGE_BUILD_TESTS=ON \
    -DSLAMFORGE_ENABLE_CERES=ON \
    -DSLAMFORGE_ENABLE_G2O=OFF
cmake --build "${project_dir}/build" --parallel

echo "BUILD COMPLETED"
