# =============================================================================
# SLAMForge Python bindings (pybind11)
# =============================================================================
#
# Build as a subdirectory of the SLAMForge project:
#   cd SLAMForge && mkdir build && cd build
#   cmake .. -DSLAMFORGE_BUILD_PYBIND=ON -DSLAMFORGE_BUILD_TESTS=OFF \
#            -DSLAMFORGE_BUILD_APPS=OFF
#   cmake --build . --target _slamforge
#
# Standalone build:
#   cmake -B build -DSLAMFORGE_ROOT=/path/to/SLAMForge && cmake --build build

# ── Locate SLAMForge ──────────────────────────────────────────────────────────
if(NOT TARGET slamforge_core)
    if(NOT DEFINED SLAMFORGE_ROOT)
        message(FATAL_ERROR "SLAMFORGE_ROOT not set. Build as a subdirectory of SLAMForge, "
                            "or pass -DSLAMFORGE_ROOT=/path/to/SLAMForge")
    endif()
    add_subdirectory(${SLAMFORGE_ROOT} ${CMAKE_BINARY_DIR}/slamforge_build)
endif()

# ── Python ─────────────────────────────────────────────────────────────────
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

# ── pybind11 ───────────────────────────────────────────────────────────────
# The top-level project normally provides pybind11.  Retain a fallback for
# standalone use of this directory without downloading a second copy when the
# target already exists.
if(NOT TARGET pybind11::module)
    find_package(pybind11 2.12 CONFIG QUIET)
endif()

if(NOT TARGET pybind11::module)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.12.0
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# ── Module target ──────────────────────────────────────────────────────────
pybind11_add_module(_slamforge
    slamforge_bindings.cpp
)

target_link_libraries(_slamforge PRIVATE
    slamforge_core
    pybind11::headers
)

target_compile_definitions(_slamforge PRIVATE
    PYBIND11_DETAILED_ERROR_MESSAGES
)

# Apply SLAMForge's project warning settings
if(COMMAND set_project_warnings)
    set_project_warnings(_slamforge)
endif()

# On Linux/macOS, hide internal symbols for smaller binary
if(UNIX AND NOT APPLE)
    target_link_options(_slamforge PRIVATE
        -Wl,--exclude-libs,ALL
    )
endif()

# Place the extension next to the Python package when installing directly with
# CMake and when scikit-build-core assembles a wheel.
install(TARGETS _slamforge
    LIBRARY DESTINATION slamforge
    RUNTIME DESTINATION slamforge
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/slamforge/__init__.py"
    DESTINATION slamforge
)
install(FILES "${PROJECT_SOURCE_DIR}/LICENSE"
    DESTINATION slamforge
)
