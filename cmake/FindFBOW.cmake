# -----------------------------------------------------------------------------
# FindFBOW.cmake — locate FBOW (Fast Bag-of-Words) library
# -----------------------------------------------------------------------------

find_path(FBOW_INCLUDE_DIR
    NAMES fbow/fbow.h
    PATHS
        /usr/local/include
        /usr/include
        ${FBOW_DIR}/include
)

find_library(FBOW_LIBRARY
    NAMES fbow
    PATHS
        /usr/local/lib
        /usr/lib
        ${FBOW_DIR}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FBOW
    REQUIRED_VARS FBOW_INCLUDE_DIR FBOW_LIBRARY
)

if(FBOW_FOUND AND NOT TARGET FBOW::FBOW)
    add_library(FBOW::FBOW UNKNOWN IMPORTED)
    set_target_properties(FBOW::FBOW PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${FBOW_INCLUDE_DIR}"
        IMPORTED_LOCATION "${FBOW_LIBRARY}"
    )
endif()
