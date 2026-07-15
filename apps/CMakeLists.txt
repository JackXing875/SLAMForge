# ── SLAMForge CLI application ──────────────────────────────────────────────────

include(GNUInstallDirs)

if(SLAMFORGE_BUILD_APPS)
    add_executable(slamforge_cli
        slamforge_cli.cpp
    )

    target_link_libraries(slamforge_cli PRIVATE
        slamforge_core
        CLI11::CLI11
    )

    # Keep installed evaluation scripts relocatable: the binary receives the
    # path from its bindir to the shared data directory instead of an absolute
    # install prefix, so `cmake --install --prefix ...` also works.
    file(RELATIVE_PATH SLAMFORGE_TOOLS_RELATIVE_TO_BINDIR
        "${CMAKE_INSTALL_FULL_BINDIR}"
        "${CMAKE_INSTALL_FULL_DATADIR}/slamforge/tools"
    )
    target_compile_definitions(slamforge_cli PRIVATE
        "SLAMFORGE_TOOLS_RELATIVE_TO_BINDIR=\"${SLAMFORGE_TOOLS_RELATIVE_TO_BINDIR}\""
    )

    # Workaround: OpenCV's highgui/imgcodecs pull in GDAL which needs curl
    find_package(CURL QUIET)
    if(CURL_FOUND)
        target_link_libraries(slamforge_cli PRIVATE CURL::libcurl)
    endif()

    set_project_warnings(slamforge_cli)

    # Install the CLI application
    install(TARGETS slamforge_cli
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/slamforge/tools"
        FILES_MATCHING
        PATTERN "*.py"
        PATTERN "__pycache__" EXCLUDE
    )
endif()

# ── ROS2 node (optional) ────────────────────────────────────────────────────
if(SLAMFORGE_BUILD_ROS2)
    add_executable(slamforge_ros_node
        slamforge_ros_node.cpp
    )
    target_link_libraries(slamforge_ros_node PRIVATE slamforge_core)
    ament_target_dependencies(slamforge_ros_node
        rclcpp
        cv_bridge
        tf2_ros
        sensor_msgs
        geometry_msgs
        visualization_msgs
    )
    target_compile_definitions(slamforge_ros_node PRIVATE SLAMFORGE_HAS_ROS2)
    set_project_warnings(slamforge_ros_node)

    install(TARGETS slamforge_ros_node
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
