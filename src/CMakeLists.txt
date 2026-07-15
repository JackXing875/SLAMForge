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
    geometry/sim3.cpp

    # Tracking
    tracking/matcher.cpp
    tracking/initializer.cpp
    tracking/tracker.cpp

    # Mapping
    mapping/local_mapper.cpp

    # Loop Closing
    loop_closing/vocabulary.cpp
    loop_closing/detector.cpp
    loop_closing/verifier.cpp
    loop_closing/corrector.cpp
    loop_closing/pose_graph.cpp
    loop_closing/global_ba.cpp
    loop_closing/loop_closing.cpp

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
    ${PROJECT_SOURCE_DIR}/include/litevo/geometry/sim3.h
    ${PROJECT_SOURCE_DIR}/include/litevo/features/orb_extractor.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/matcher.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/initializer.h
    ${PROJECT_SOURCE_DIR}/include/litevo/tracking/tracker.h
    ${PROJECT_SOURCE_DIR}/include/litevo/mapping/local_mapper.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/vocabulary.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/detector.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/verifier.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/corrector.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/pose_graph.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/global_ba.h
    ${PROJECT_SOURCE_DIR}/include/litevo/loop_closing/loop_closing.h
    ${PROJECT_SOURCE_DIR}/include/litevo/optimization/ba.h
)

# Create the core library target
add_library(litevo_core STATIC
    ${LITEVO_CORE_SOURCES}
    ${LITEVO_CORE_HEADERS}
)

# The Python extension links litevo_core into a shared module.  Building the
# archive as PIC keeps that link valid on ELF and Mach-O platforms.
if(LITEVO_BUILD_PYBIND)
    set_target_properties(litevo_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Include directories
target_include_directories(litevo_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

# This is part of the installed target's public build contract.  Setting only
# CMAKE_CXX_STANDARD at the top level does not tell downstream consumers that
# the public headers require C++20 facilities.
target_compile_features(litevo_core PUBLIC cxx_std_20)

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

if(LITEVO_ENABLE_YAML)
    # yaml-cpp 0.7 packages commonly export `yaml-cpp`, while newer builds
    # export `yaml-cpp::yaml-cpp`.  Support both so distro packages enable the
    # configuration system instead of silently compiling its fallback path.
    if(TARGET yaml-cpp::yaml-cpp)
        target_link_libraries(litevo_core PUBLIC yaml-cpp::yaml-cpp)
        target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_YAML_CPP)
    elseif(TARGET yaml-cpp)
        target_link_libraries(litevo_core PUBLIC yaml-cpp)
        target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_YAML_CPP)
    endif()
endif()

if(LITEVO_ENABLE_CERES AND TARGET Ceres::ceres)
    target_link_libraries(litevo_core PUBLIC Ceres::ceres)
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_CERES)
endif()

if(LITEVO_ENABLE_G2O)
    # Installed g2o exports namespaced targets, while its FetchContent build
    # exposes the unqualified target names.  Link actual CMake targets rather
    # than their libg2o_ output filenames so transitive dependencies (notably
    # g2o_stuff) are retained on the final application link line.
    if(TARGET g2o::core)
        target_link_libraries(litevo_core PUBLIC
            g2o::core g2o::types_slam3d g2o::solver_eigen g2o::csparse_extension g2o::stuff
        )
    elseif(TARGET core)
        target_link_libraries(litevo_core PUBLIC
            core types_slam3d solver_eigen csparse_extension stuff
        )
    else()
        message(FATAL_ERROR "LITEVO_ENABLE_G2O is ON but no usable g2o targets were found")
    endif()
    target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_G2O)
endif()

if(LITEVO_ENABLE_FBOW)
    find_package(FBOW QUIET)
    if(NOT FBOW_FOUND)
        include(FetchContent)
        FetchContent_Declare(fbow
            GIT_REPOSITORY  https://github.com/rmsalinas/fbow.git
            GIT_TAG         master
        )
        FetchContent_MakeAvailable(fbow)
    endif()
    if(TARGET FBOW::FBOW)
        target_link_libraries(litevo_core PUBLIC FBOW::FBOW)
        target_compile_definitions(litevo_core PUBLIC LITEVO_HAS_FBOW)
    endif()
endif()
