# ── LiteVO core library ─────────────────────────────────────────────────────

# Configure the version header from template
configure_file(
    "${PROJECT_SOURCE_DIR}/include/litevo/version.h.in"
    "${PROJECT_BINARY_DIR}/include/litevo/version.h"
)

# ── Collect all library sources ──────────────────────────────────────────────

set(LITEVO_CORE_SOURCES
    # Core
    version.cpp
    core/camera.cpp
    core/config.cpp
    core/feature_grid.cpp
    core/frame.cpp
    core/keyframe.cpp
    core/map_point.cpp
    core/map.cpp

    # Features
    features/orb_extractor.cpp

    # Geometry
    geometry/se3.cpp
    geometry/triangulation.cpp
    geometry/epipolar.cpp
    geometry/pnp.cpp

    # Tracking
    tracking/matcher.cpp
    tracking/initializer.cpp
    tracking/tracker.cpp

    # Mapping
    mapping/local_mapper.cpp

    # Optimization
    optimization/ba.cpp
)

set(LITEVO_CORE_HEADERS
    ${PROJECT_SOURCE_DIR}/include/litevo/litevo.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/types.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/config.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/camera.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/feature_grid.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/frame.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/keyframe.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/map_point.h
    ${PROJECT_SOURCE_DIR}/include/litevo/core/map.h
    ${PROJECT_SOURCE_DIR}/include/litevo/base/exception.h
    ${PROJECT_SOURCE_DIR}/include/litevo/geometry/se3.h
    ${PROJECT_SOURCE_DIR}/include/litevo/geometry/triangulation.h
    ${PROJECT_SOURCE_DIR}/include/litevo/geometry/epipolar.h
    ${PROJECT_SOURCE_DIR}/include/litevo/geometry/pnp.h
    ${PROJECT_SOURCE_DIR}/include/litevo/features/orb_extractor.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/matcher.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/initializer.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/tracker.h
    ${PROJECT_SOURCE_DIR}/include/litevo/mapping/local_mapper.h
    ${PROJECT_SOURCE_DIR}/include/litevo/optimization/ba.h
)

# Create the core library target
add_library(litevo_core STATIC
    ${LITEVO_CORE_SOURCES}
    ${LITEVO_CORE_HEADERS}
)

# Include directories
target_include_directories(litevo_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

# Compiler warnings
set_project_warnings(litevo_core)

# Link libraries
target_link_libraries(litevo_core
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

# Optional dependencies
if(LITEVO_ENABLE_SOPHUS AND TARGET Sophus::Sophus)
    target_link_libraries(litevo_core PUBLIC Sophus::Sophus)
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_SOPHUS)
endif()

if(LITEVO_ENABLE_SPDLOG AND TARGET spdlog::spdlog)
    target_link_libraries(litevo_core PUBLIC spdlog::spdlog)
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_SPDLOG)
endif()

if(LITEVO_ENABLE_YAML AND TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(litevo_core PUBLIC yaml-cpp::yaml-cpp)
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_YAML_CPP)
endif()

if(LITEVO_ENABLE_CERES AND TARGET Ceres::ceres)
    target_link_libraries(litevo_core PUBLIC Ceres::ceres)
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_CERES)
endif()

if(LITEVO_ENABLE_G2O)
    target_link_libraries(litevo_core PUBLIC
        g2o_core g2o_types_slam3d g2o_solver_eigen g2o_csparse_extension
    )
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_G2O)
endif()
