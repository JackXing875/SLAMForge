# =============================================================================
# SLAMForge — Monocular Visual SLAM and Dense Reconstruction
# =============================================================================
cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# ── Project identity ─────────────────────────────────────────────────────────
project(SLAMForge
    VERSION 3.2.0
    DESCRIPTION "Monocular visual SLAM and dense reconstruction system"
    HOMEPAGE_URL "https://github.com/JackXing875/SLAMForge"
    LANGUAGES CXX
)

# CMake package compatibility uses the numeric project version, while public
# beta builds expose the complete prerelease identifier in the CLI and GUI.
set(SLAMFORGE_VERSION_PRERELEASE "beta.2")
set(SLAMFORGE_VERSION_STRING "${PROJECT_VERSION}-${SLAMFORGE_VERSION_PRERELEASE}")

# ── C++ standard ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build options ────────────────────────────────────────────────────────────
option(SLAMFORGE_BUILD_TESTS   "Build unit and integration tests"     ON)
option(SLAMFORGE_BUILD_PYTHON_TESTS "Build Python trajectory-tool regressions" ON)
option(SLAMFORGE_BUILD_APPS    "Build CLI applications"               ON)
option(SLAMFORGE_BUILD_DESKTOP "Build the Qt 6 desktop application"   OFF)
option(SLAMFORGE_ENABLE_SOPHUS "Enable Sophus Lie algebra support"    ON)
option(SLAMFORGE_ENABLE_CERES  "Enable Ceres Solver optimization"     ON)
option(SLAMFORGE_ENABLE_G2O    "Enable g2o pose graph optimization"   ON)
option(SLAMFORGE_ENABLE_FBOW   "Enable FBOW visual vocabulary"         ON)
option(SLAMFORGE_ENABLE_SPDLOG "Enable spdlog structured logging"     ON)
option(SLAMFORGE_ENABLE_YAML   "Enable YAML configuration support"    ON)
option(SLAMFORGE_ENABLE_DENSE  "Enable ONNX Runtime dense reconstruction" ON)
option(SLAMFORGE_ENABLE_IPO    "Enable inter-procedural optimization" OFF)
option(SLAMFORGE_BUILD_PYBIND     "Build pybind11 Python bindings"       OFF)
option(SLAMFORGE_BUILD_ROS2       "Build ROS2 integration"               OFF)
option(SLAMFORGE_BUILD_BENCHMARKS "Build performance benchmarks"        OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Root of an official ONNX Runtime C/C++ package")
set(SLAMFORGE_DEPTH_MODEL_FILE "" CACHE FILEPATH
    "Depth Anything V2 Small ONNX model installed with desktop packages")

# ── Inter-procedural optimization ────────────────────────────────────────────
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported AND SLAMFORGE_ENABLE_IPO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# ── Compiler warnings ────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
include(CompilerWarnings)

# ── Dependencies ─────────────────────────────────────────────────────────────
# Required: OpenCV (image I/O, feature extraction, geometry)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d video videoio)

# Required: Eigen3 (linear algebra)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Optional: ONNX Runtime powers offline dense depth inference. It is kept
# optional for lightweight library/CI builds and enabled in desktop releases.
if(SLAMFORGE_ENABLE_DENSE)
    find_package(ONNXRuntime QUIET)
    if(NOT ONNXRuntime_FOUND)
        message(WARNING "ONNX Runtime not found — dense reconstruction disabled")
        set(SLAMFORGE_ENABLE_DENSE OFF)
    endif()
endif()

# Optional: Sophus (SE3 Lie algebra — highly recommended)
if(SLAMFORGE_ENABLE_SOPHUS)
    find_package(Sophus QUIET)
    if(NOT Sophus_FOUND)
        message(STATUS "Sophus not found — will build from FetchContent")
        include(FetchContent)
        FetchContent_Declare(Sophus
            GIT_REPOSITORY  https://github.com/strasdat/Sophus.git
            GIT_TAG         v1.22.10
        )
        FetchContent_MakeAvailable(Sophus)
        set(Sophus_FOUND TRUE)
    endif()
endif()

# Optional: Ceres Solver (bundle adjustment)
if(SLAMFORGE_ENABLE_CERES)
    find_package(Ceres QUIET)
    if(NOT Ceres_FOUND)
        message(WARNING "Ceres Solver not found — backend optimization disabled")
        set(SLAMFORGE_ENABLE_CERES OFF)
    endif()
endif()

# Optional: g2o (pose graph optimization)
if(SLAMFORGE_ENABLE_G2O)
    # g2o has used both g2oConfig.cmake and G2OConfig.cmake across releases.
    # Probe both spellings before falling back to a source build so an installed
    # dependency is not needlessly rebuilt.
    find_package(g2o CONFIG QUIET)
    if(NOT TARGET g2o::core AND NOT TARGET core)
        find_package(G2O CONFIG QUIET)
    endif()

    if(NOT TARGET g2o::core AND NOT TARGET core)
        message(STATUS "g2o not found — trying FetchContent fallback")
        include(FetchContent)
        FetchContent_Declare(g2o
            GIT_REPOSITORY  https://github.com/RainerKuemmerle/g2o.git
            GIT_TAG         20230223_git
        )
        set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
        set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(g2o)
    endif()

    # The g2o source tree uses unqualified targets (for example `core`),
    # whereas its installed package exports `g2o::core`.  Normalize the
    # in-tree form so SLAMForge always records portable namespaced targets in
    # its public link interface and in its exported CMake package.
    foreach(g2o_component core types_slam3d solver_eigen csparse_extension stuff)
        if(TARGET ${g2o_component} AND NOT TARGET g2o::${g2o_component})
            add_library(g2o::${g2o_component} ALIAS ${g2o_component})
        endif()
    endforeach()

    if(NOT TARGET g2o::core)
        message(WARNING "g2o targets were not created — pose graph optimization disabled")
        set(SLAMFORGE_ENABLE_G2O OFF)
    endif()
endif()

# Optional: spdlog (structured logging)
if(SLAMFORGE_ENABLE_SPDLOG)
    find_package(spdlog QUIET)
    if(NOT spdlog_FOUND)
        include(FetchContent)
        FetchContent_Declare(spdlog
            GIT_REPOSITORY  https://github.com/gabime/spdlog.git
            GIT_TAG         v1.13.0
        )
        FetchContent_MakeAvailable(spdlog)
        set(spdlog_FOUND TRUE)
    endif()
endif()

# Optional: yaml-cpp (configuration files)
if(SLAMFORGE_ENABLE_YAML)
    find_package(yaml-cpp QUIET)
    if(NOT yaml-cpp_FOUND)
        include(FetchContent)
        FetchContent_Declare(yaml-cpp
            GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp.git
            GIT_TAG         yaml-cpp-0.8.0
        )
        FetchContent_MakeAvailable(yaml-cpp)
        set(yaml-cpp_FOUND TRUE)
    endif()
endif()

# Test framework: GoogleTest
if(SLAMFORGE_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY  https://github.com/google/googletest.git
        GIT_TAG         v1.14.0
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
endif()

# CLI library: CLI11
if(SLAMFORGE_BUILD_APPS)
    include(FetchContent)
    FetchContent_Declare(cli11
        GIT_REPOSITORY  https://github.com/CLIUtils/CLI11.git
        GIT_TAG         v2.4.1
    )
    FetchContent_MakeAvailable(cli11)
endif()

# Qt is required only for the end-user desktop application. Keep this opt-in
# while the desktop packaging matrix is being established so existing library,
# CLI, ROS2, and Python builds remain dependency-compatible.
if(SLAMFORGE_BUILD_DESKTOP)
    if(NOT SLAMFORGE_BUILD_APPS)
        message(FATAL_ERROR
            "SLAMFORGE_BUILD_DESKTOP currently requires SLAMFORGE_BUILD_APPS because "
            "the first desktop milestone runs the verified slamforge_cli pipeline")
    endif()
    find_package(Qt6 6.2 REQUIRED COMPONENTS Widgets)
endif()

# Python bindings: pybind11
if(SLAMFORGE_BUILD_PYBIND)
    # A PEP 517 build supplies pybind11 through its build requirements.  Keep
    # FetchContent as a convenience fallback for direct CMake users, but do
    # not download it when an installed config package is already available.
    find_package(pybind11 2.12 CONFIG QUIET)
    if(NOT TARGET pybind11::module)
        include(FetchContent)
        FetchContent_Declare(pybind11
            GIT_REPOSITORY  https://github.com/pybind/pybind11.git
            GIT_TAG         v2.12.0
        )
        FetchContent_MakeAvailable(pybind11)
    endif()
endif()

# ROS2 integration
if(SLAMFORGE_BUILD_ROS2)
    find_package(ament_cmake QUIET)
    find_package(rclcpp QUIET)
    find_package(cv_bridge QUIET)
    find_package(tf2_ros QUIET)
    find_package(sensor_msgs QUIET)
    find_package(geometry_msgs QUIET)
    find_package(visualization_msgs QUIET)
    if(ament_cmake_FOUND AND rclcpp_FOUND AND cv_bridge_FOUND AND tf2_ros_FOUND
       AND sensor_msgs_FOUND AND geometry_msgs_FOUND AND visualization_msgs_FOUND)
        message(STATUS "ROS2 packages found — enabling ROS2 integration")
    else()
        message(WARNING "Required ROS2 packages not found — disabling ROS2 integration")
        set(SLAMFORGE_BUILD_ROS2 OFF)
    endif()
endif()

# Performance benchmarks: Google Benchmark
if(SLAMFORGE_BUILD_BENCHMARKS)
    include(FetchContent)
    FetchContent_Declare(benchmark
        GIT_REPOSITORY  https://github.com/google/benchmark.git
        GIT_TAG         v1.9.0
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
endif()

# ── Library target ───────────────────────────────────────────────────────────
add_subdirectory(src)
if(SLAMFORGE_BUILD_APPS OR SLAMFORGE_BUILD_DESKTOP OR SLAMFORGE_BUILD_ROS2)
    add_subdirectory(apps)
endif()

if(SLAMFORGE_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(SLAMFORGE_BUILD_PYBIND)
    add_subdirectory(pybind)
endif()

if(SLAMFORGE_BUILD_BENCHMARKS)
    add_subdirectory(tests/bench)
endif()

# ── Installation ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Ship dataset presets with the installed library and CLI.  They remain
# examples rather than hidden runtime state, so callers can copy and tune them.
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/slamforge/config"
    COMPONENT Desktop
    FILES_MATCHING PATTERN "*.yaml"
)

install(TARGETS slamforge_core
    EXPORT  SLAMForgeTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/slamforge
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN "version.h.in" EXCLUDE
)

# version.h is generated into the build tree, so it is not covered by the
# source-header directory install above.  Installing it explicitly makes the
# public umbrella header usable from an installed prefix.
install(FILES "${PROJECT_BINARY_DIR}/include/slamforge/version.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/slamforge
)

# GPL distributions must carry the license text with every installed package.
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/SLAMForge"
    COMPONENT Desktop
)
install(FILES "${PROJECT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/SLAMForge"
    COMPONENT Desktop
)
install(FILES "${PROJECT_SOURCE_DIR}/packaging/licenses/Depth-Anything-V2.txt"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/SLAMForge/third-party"
    COMPONENT Desktop
)
if(SLAMFORGE_ENABLE_DENSE AND ONNXRUNTIME_RUNTIME_LIBRARY)
    if(EXISTS "${ONNXRUNTIME_ROOT}/LICENSE")
        install(FILES "${ONNXRUNTIME_ROOT}/LICENSE"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/SLAMForge/third-party"
            RENAME "ONNX-Runtime.txt"
            COMPONENT Desktop)
    endif()
    install(DIRECTORY "${ONNXRUNTIME_INCLUDE_DIR}/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/onnxruntime"
        COMPONENT Development
    )
    if(WIN32)
        install(FILES "${ONNXRUNTIME_RUNTIME_LIBRARY}"
            DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Desktop)
    else()
        file(GLOB ONNXRUNTIME_RUNTIME_FILES
            "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so*"
        )
        install(FILES ${ONNXRUNTIME_RUNTIME_FILES}
            DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Desktop)
    endif()
endif()
if(SLAMFORGE_DEPTH_MODEL_FILE AND EXISTS "${SLAMFORGE_DEPTH_MODEL_FILE}")
    install(FILES "${SLAMFORGE_DEPTH_MODEL_FILE}"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/slamforge/models"
        RENAME "depth_anything_v2_vits_dynamic.onnx"
        COMPONENT Desktop)
endif()

install(EXPORT SLAMForgeTargets
    FILE SLAMForgeTargets.cmake
    NAMESPACE SLAMForge::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SLAMForge
)

# Record the dependencies that are part of slamforge_core's public link interface.
# The generated config file recreates those imported targets for downstream
# projects before it includes SLAMForgeTargets.cmake.
set(SLAMFORGE_PACKAGE_WITH_SOPHUS OFF)
if(SLAMFORGE_ENABLE_SOPHUS AND TARGET Sophus::Sophus)
    set(SLAMFORGE_PACKAGE_WITH_SOPHUS ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_CERES OFF)
if(SLAMFORGE_ENABLE_CERES AND TARGET Ceres::ceres)
    set(SLAMFORGE_PACKAGE_WITH_CERES ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_G2O OFF)
if(SLAMFORGE_ENABLE_G2O AND (TARGET g2o::core OR TARGET core))
    set(SLAMFORGE_PACKAGE_WITH_G2O ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_FBOW OFF)
if(SLAMFORGE_ENABLE_FBOW AND TARGET FBOW::FBOW)
    set(SLAMFORGE_PACKAGE_WITH_FBOW ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_SPDLOG OFF)
if(SLAMFORGE_ENABLE_SPDLOG AND TARGET spdlog::spdlog)
    set(SLAMFORGE_PACKAGE_WITH_SPDLOG ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_YAML OFF)
if(SLAMFORGE_ENABLE_YAML AND (TARGET yaml-cpp::yaml-cpp OR TARGET yaml-cpp))
    set(SLAMFORGE_PACKAGE_WITH_YAML ON)
endif()

set(SLAMFORGE_PACKAGE_WITH_ONNXRUNTIME OFF)
if(SLAMFORGE_ENABLE_DENSE AND TARGET ONNXRuntime::ONNXRuntime)
    set(SLAMFORGE_PACKAGE_WITH_ONNXRUNTIME ON)
endif()

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/SLAMForgeConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/SLAMForgeConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SLAMForge"
)

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/SLAMForgeConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${PROJECT_BINARY_DIR}/SLAMForgeConfig.cmake"
    "${PROJECT_BINARY_DIR}/SLAMForgeConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SLAMForge"
)

# These fallback find modules are needed only when a dependency does not ship a
# CMake package config (notably FBOW), but shipping them keeps a SLAMForge package
# self-contained for consumers using a conventional system installation.
install(FILES
    "${PROJECT_SOURCE_DIR}/cmake/FindSophus.cmake"
    "${PROJECT_SOURCE_DIR}/cmake/FindFBOW.cmake"
    "${PROJECT_SOURCE_DIR}/cmake/FindONNXRuntime.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SLAMForge/modules"
)

# ── Packaging (CPack) ────────────────────────────────────────────────────────
set(CPACK_PACKAGE_NAME "SLAMForge")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Monocular visual SLAM and dense reconstruction system")
set(CPACK_PACKAGE_HOMEPAGE_URL "${PROJECT_HOMEPAGE_URL}")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "SLAMForge Team")
include(CPack)

# ── Feature summary ──────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "  SLAMForge ${SLAMFORGE_VERSION_STRING} — Configuration Summary")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "  C++ Standard:       ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build tests:        ${SLAMFORGE_BUILD_TESTS}")
message(STATUS "  Build apps:         ${SLAMFORGE_BUILD_APPS}")
message(STATUS "  Desktop app:        ${SLAMFORGE_BUILD_DESKTOP}")
message(STATUS "  Sophus (SE3):       ${SLAMFORGE_ENABLE_SOPHUS}")
message(STATUS "  Ceres (BA):         ${SLAMFORGE_ENABLE_CERES}")
message(STATUS "  g2o (Pose Graph):   ${SLAMFORGE_ENABLE_G2O}")
message(STATUS "  FBOW (Vocabulary):  ${SLAMFORGE_ENABLE_FBOW}")
message(STATUS "  spdlog (Logging):   ${SLAMFORGE_ENABLE_SPDLOG}")
message(STATUS "  yaml-cpp (Config):  ${SLAMFORGE_ENABLE_YAML}")
message(STATUS "  Dense mapping:      ${SLAMFORGE_ENABLE_DENSE}")
message(STATUS "  Python bindings:    ${SLAMFORGE_BUILD_PYBIND}")
message(STATUS "  ROS2 integration:   ${SLAMFORGE_BUILD_ROS2}")
message(STATUS "  Benchmarks:         ${SLAMFORGE_BUILD_BENCHMARKS}")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "")
