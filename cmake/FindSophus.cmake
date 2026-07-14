# -----------------------------------------------------------------------------
# FindSophus.cmake — locate Sophus Lie algebra library
# -----------------------------------------------------------------------------

find_path(Sophus_INCLUDE_DIR
    NAMES sophus/se3.hpp
    PATHS
        /usr/local/include
        /usr/include
        ${Sophus_DIR}/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sophus
    REQUIRED_VARS Sophus_INCLUDE_DIR
    VERSION_VAR Sophus_VERSION
)

if(Sophus_FOUND AND NOT TARGET Sophus::Sophus)
    add_library(Sophus::Sophus INTERFACE IMPORTED)
    set_target_properties(Sophus::Sophus PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Sophus_INCLUDE_DIR}"
    )
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    find_package(fmt REQUIRED)
    target_link_libraries(Sophus::Sophus INTERFACE Eigen3::Eigen fmt::fmt)
endif()
