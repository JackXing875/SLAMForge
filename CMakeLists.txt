# =============================================================================
# LiteVO — Industrial-Grade Monocular Visual SLAM System
# =============================================================================
cmake_minimum_required(VERSION 3.20)

# ── Project identity ─────────────────────────────────────────────────────────
project(litevo
    VERSION 2.0.0
    DESCRIPTION "Industrial-grade monocular visual SLAM system"
    LANGUAGES CXX
)

# ── C++ standard ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build options ────────────────────────────────────────────────────────────
option(LITEVO_BUILD_TESTS   "Build unit and integration tests"     ON)
option(LITEVO_BUILD_PYTHON_TESTS "Build Python trajectory-tool regressions" ON)
option(LITEVO_BUILD_APPS    "Build CLI applications"               ON)
option(LITEVO_ENABLE_SOPHUS "Enable Sophus Lie algebra support"    ON)
option(LITEVO_ENABLE_CERES  "Enable Ceres Solver optimization"     ON)
option(LITEVO_ENABLE_G2O    "Enable g2o pose graph optimization"   ON)
option(LITEVO_ENABLE_FBOW   "Enable FBOW visual vocabulary"         ON)
option(LITEVO_ENABLE_SPDLOG "Enable spdlog structured logging"     ON)
option(LITEVO_ENABLE_YAML   "Enable YAML configuration support"    ON)
option(LITEVO_ENABLE_IPO    "Enable inter-procedural optimization" OFF)
option(LITEVO_BUILD_PYBIND     "Build pybind11 Python bindings"       OFF)
option(LITEVO_BUILD_ROS2       "Build ROS2 integration"               OFF)
option(LITEVO_BUILD_BENCHMARKS "Build performance benchmarks"        OFF)

# ── Inter-procedural optimization ────────────────────────────────────────────
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported AND LITEVO_ENABLE_IPO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# ── Compiler warnings ────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
include(CompilerWarnings)

# ── Dependencies ─────────────────────────────────────────────────────────────
# Required: OpenCV (image I/O, feature extraction, geometry)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d videoio)

# Required: Eigen3 (linear algebra)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Optional: Sophus (SE3 Lie algebra — highly recommended)
if(LITEVO_ENABLE_SOPHUS)
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
if(LITEVO_ENABLE_CERES)
    find_package(Ceres QUIET)
    if(NOT Ceres_FOUND)
        message(WARNING "Ceres Solver not found — backend optimization disabled")
        set(LITEVO_ENABLE_CERES OFF)
    endif()
endif()

# Optional: g2o (pose graph optimization)
if(LITEVO_ENABLE_G2O)
    find_package(g2o QUIET)
    if(NOT g2o_FOUND)
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
endif()

# Optional: spdlog (structured logging)
if(LITEVO_ENABLE_SPDLOG)
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
if(LITEVO_ENABLE_YAML)
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
if(LITEVO_BUILD_TESTS)
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
if(LITEVO_BUILD_APPS)
    include(FetchContent)
    FetchContent_Declare(cli11
        GIT_REPOSITORY  https://github.com/CLIUtils/CLI11.git
        GIT_TAG         v2.4.1
    )
    FetchContent_MakeAvailable(cli11)
endif()

# Python bindings: pybind11
if(LITEVO_BUILD_PYBIND)
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
if(LITEVO_BUILD_ROS2)
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
        set(LITEVO_BUILD_ROS2 OFF)
    endif()
endif()

# Performance benchmarks: Google Benchmark
if(LITEVO_BUILD_BENCHMARKS)
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
if(LITEVO_BUILD_APPS OR LITEVO_BUILD_ROS2)
    add_subdirectory(apps)
endif()

if(LITEVO_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(LITEVO_BUILD_PYBIND)
    add_subdirectory(pybind)
endif()

if(LITEVO_BUILD_BENCHMARKS)
    add_subdirectory(tests/bench)
endif()

# ── Installation ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Ship dataset presets with the installed library and CLI.  They remain
# examples rather than hidden runtime state, so callers can copy and tune them.
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/litevo/config"
    FILES_MATCHING PATTERN "*.yaml"
)

install(TARGETS litevo_core
    EXPORT  litevoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/litevo
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN "version.h.in" EXCLUDE
)

# version.h is generated into the build tree, so it is not covered by the
# source-header directory install above.  Installing it explicitly makes the
# public umbrella header usable from an installed prefix.
install(FILES "${PROJECT_BINARY_DIR}/include/litevo/version.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/litevo
)

install(EXPORT litevoTargets
    FILE litevoTargets.cmake
    NAMESPACE litevo::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litevo
)

# Record the dependencies that are part of litevo_core's public link interface.
# The generated config file recreates those imported targets for downstream
# projects before it includes litevoTargets.cmake.
set(LITEVO_PACKAGE_WITH_SOPHUS OFF)
if(LITEVO_ENABLE_SOPHUS AND TARGET Sophus::Sophus)
    set(LITEVO_PACKAGE_WITH_SOPHUS ON)
endif()

set(LITEVO_PACKAGE_WITH_CERES OFF)
if(LITEVO_ENABLE_CERES AND TARGET Ceres::ceres)
    set(LITEVO_PACKAGE_WITH_CERES ON)
endif()

set(LITEVO_PACKAGE_WITH_G2O OFF)
if(LITEVO_ENABLE_G2O AND TARGET g2o_core)
    set(LITEVO_PACKAGE_WITH_G2O ON)
endif()

set(LITEVO_PACKAGE_WITH_FBOW OFF)
if(LITEVO_ENABLE_FBOW AND TARGET FBOW::FBOW)
    set(LITEVO_PACKAGE_WITH_FBOW ON)
endif()

set(LITEVO_PACKAGE_WITH_SPDLOG OFF)
if(LITEVO_ENABLE_SPDLOG AND TARGET spdlog::spdlog)
    set(LITEVO_PACKAGE_WITH_SPDLOG ON)
endif()

set(LITEVO_PACKAGE_WITH_YAML OFF)
if(LITEVO_ENABLE_YAML AND (TARGET yaml-cpp::yaml-cpp OR TARGET yaml-cpp))
    set(LITEVO_PACKAGE_WITH_YAML ON)
endif()

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/litevoConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/litevoConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/litevo"
)

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/litevoConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${PROJECT_BINARY_DIR}/litevoConfig.cmake"
    "${PROJECT_BINARY_DIR}/litevoConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/litevo"
)

# These fallback find modules are needed only when a dependency does not ship a
# CMake package config (notably FBOW), but shipping them keeps a LiteVO package
# self-contained for consumers using a conventional system installation.
install(FILES
    "${PROJECT_SOURCE_DIR}/cmake/FindSophus.cmake"
    "${PROJECT_SOURCE_DIR}/cmake/FindFBOW.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/litevo/modules"
)

# ── Packaging (CPack) ────────────────────────────────────────────────────────
set(CPACK_PACKAGE_NAME "litevo")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Industrial-grade monocular visual SLAM system")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "LiteVO Team")
include(CPack)

# ── Feature summary ──────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "  LiteVO ${PROJECT_VERSION} — Configuration Summary")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "  C++ Standard:       ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build tests:        ${LITEVO_BUILD_TESTS}")
message(STATUS "  Build apps:         ${LITEVO_BUILD_APPS}")
message(STATUS "  Sophus (SE3):       ${LITEVO_ENABLE_SOPHUS}")
message(STATUS "  Ceres (BA):         ${LITEVO_ENABLE_CERES}")
message(STATUS "  g2o (Pose Graph):   ${LITEVO_ENABLE_G2O}")
message(STATUS "  FBOW (Vocabulary):  ${LITEVO_ENABLE_FBOW}")
message(STATUS "  spdlog (Logging):   ${LITEVO_ENABLE_SPDLOG}")
message(STATUS "  yaml-cpp (Config):  ${LITEVO_ENABLE_YAML}")
message(STATUS "  Python bindings:    ${LITEVO_BUILD_PYBIND}")
message(STATUS "  ROS2 integration:   ${LITEVO_BUILD_ROS2}")
message(STATUS "  Benchmarks:         ${LITEVO_BUILD_BENCHMARKS}")
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "")
