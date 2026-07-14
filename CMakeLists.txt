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
option(LITEVO_BUILD_APPS    "Build CLI applications"               ON)
option(LITEVO_ENABLE_SOPHUS "Enable Sophus Lie algebra support"    ON)
option(LITEVO_ENABLE_CERES  "Enable Ceres Solver optimization"     ON)
option(LITEVO_ENABLE_G2O    "Enable g2o pose graph optimization"   ON)
option(LITEVO_ENABLE_FBOW   "Enable FBOW visual vocabulary"         ON)
option(LITEVO_ENABLE_SPDLOG "Enable spdlog structured logging"     ON)
option(LITEVO_ENABLE_YAML   "Enable YAML configuration support"    ON)
option(LITEVO_ENABLE_IPO    "Enable inter-procedural optimization" OFF)

# ── Inter-procedural optimization ────────────────────────────────────────────
include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported AND LITEVO_ENABLE_IPO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# ── Compiler warnings ────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
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

# ── Library target ───────────────────────────────────────────────────────────
add_subdirectory(src)
if(LITEVO_BUILD_APPS)
    add_subdirectory(apps)
endif()

if(LITEVO_BUILD_TESTS)
    add_subdirectory(tests)
endif()

# ── Installation ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS litevo_core
    EXPORT  litevoTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/litevo
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT litevoTargets
    FILE litevoTargets.cmake
    NAMESPACE litevo::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/litevo
)

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
message(STATUS "═══════════════════════════════════════════════════════════════")
message(STATUS "")
