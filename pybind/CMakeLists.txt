# =============================================================================
# LiteVO Python bindings (pybind11)
# =============================================================================
#
# Build as a subdirectory of the LiteVO project:
#   cd LiteVO && mkdir build && cd build
#   cmake .. -DLITEVO_BUILD_TESTS=OFF -DLITEVO_BUILD_APPS=OFF
#   cmake --build . --target _litevo
#
# Standalone build:
#   cmake -B build -DLITEVO_ROOT=/path/to/LiteVO && cmake --build build

# ── Locate LiteVO ──────────────────────────────────────────────────────────
if(NOT TARGET litevo_core)
    if(NOT DEFINED LITEVO_ROOT)
        message(FATAL_ERROR "LITEVO_ROOT not set. Build as a subdirectory of LiteVO, "
                            "or pass -DLITEVO_ROOT=/path/to/LiteVO")
    endif()
    add_subdirectory(${LITEVO_ROOT} ${CMAKE_BINARY_DIR}/litevo_build)
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
pybind11_add_module(_litevo
    litevo_bindings.cpp
)

target_link_libraries(_litevo PRIVATE
    litevo_core
    pybind11::headers
)

target_compile_definitions(_litevo PRIVATE
    PYBIND11_DETAILED_ERROR_MESSAGES
)

# Apply LiteVO's project warning settings
if(COMMAND set_project_warnings)
    set_project_warnings(_litevo)
endif()

# On Linux/macOS, hide internal symbols for smaller binary
if(UNIX AND NOT APPLE)
    target_link_options(_litevo PRIVATE
        -Wl,--exclude-libs,ALL
    )
endif()

# Place the extension next to the Python package when installing directly with
# CMake and when scikit-build-core assembles a wheel.
install(TARGETS _litevo
    LIBRARY DESTINATION litevo
    RUNTIME DESTINATION litevo
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/litevo/__init__.py"
    DESTINATION litevo
)
