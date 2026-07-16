#pragma once

#ifdef SLAMFORGE_HAS_CERES

#include <ceres/ceres.h>
#include <ceres/version.h>

namespace slamforge::optimization::detail {

inline void AddQuaternionParameterBlock(ceres::Problem& problem, double* quaternion) {
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
    problem.AddParameterBlock(quaternion, 4, new ceres::QuaternionManifold());
#else
    problem.AddParameterBlock(quaternion, 4, new ceres::QuaternionParameterization());
#endif
}

}  // namespace slamforge::optimization::detail

#endif  // SLAMFORGE_HAS_CERES
