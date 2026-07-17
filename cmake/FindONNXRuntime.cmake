# Find an official ONNX Runtime C/C++ binary package.
#
# Inputs:
#   ONNXRUNTIME_ROOT
#
# Outputs:
#   ONNXRuntime_FOUND
#   ONNXRUNTIME_INCLUDE_DIR
#   ONNXRUNTIME_LIBRARY
#   ONNXRUNTIME_RUNTIME_LIBRARY
#   ONNXRuntime::ONNXRuntime

find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    HINTS "${ONNXRUNTIME_ROOT}"
    PATH_SUFFIXES include include/onnxruntime
)

find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS "${ONNXRUNTIME_ROOT}"
    PATH_SUFFIXES lib lib64
)

if(WIN32)
    find_file(ONNXRUNTIME_RUNTIME_LIBRARY
        NAMES onnxruntime.dll
        HINTS "${ONNXRUNTIME_ROOT}"
        PATH_SUFFIXES lib bin
    )
else()
    set(ONNXRUNTIME_RUNTIME_LIBRARY "${ONNXRUNTIME_LIBRARY}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ONNXRuntime
    REQUIRED_VARS ONNXRUNTIME_INCLUDE_DIR ONNXRUNTIME_LIBRARY ONNXRUNTIME_RUNTIME_LIBRARY
)

if(ONNXRuntime_FOUND AND NOT TARGET ONNXRuntime::ONNXRuntime)
    add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
    )
    if(WIN32)
        set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
            IMPORTED_IMPLIB "${ONNXRUNTIME_LIBRARY}"
            IMPORTED_LOCATION "${ONNXRUNTIME_RUNTIME_LIBRARY}"
        )
    else()
        set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
            IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
        )
    endif()
endif()

mark_as_advanced(ONNXRUNTIME_INCLUDE_DIR ONNXRUNTIME_LIBRARY ONNXRUNTIME_RUNTIME_LIBRARY)
