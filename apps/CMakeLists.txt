# ── LiteVO CLI application ──────────────────────────────────────────────────

include(GNUInstallDirs)

if(LITEVO_BUILD_APPS)
    add_executable(litevo_cli
        litevo_cli.cpp
    )

    target_link_libraries(litevo_cli PRIVATE
        litevo_core
        CLI11::CLI11
    )

    # Keep installed evaluation scripts relocatable: the binary receives the
    # path from its bindir to the shared data directory instead of an absolute
    # install prefix, so `cmake --install --prefix ...` also works.
    file(RELATIVE_PATH LITEVO_TOOLS_RELATIVE_TO_BINDIR
        "${CMAKE_INSTALL_FULL_BINDIR}"
        "${CMAKE_INSTALL_FULL_DATADIR}/litevo/tools"
    )
    target_compile_definitions(litevo_cli PRIVATE
        "LITEVO_TOOLS_RELATIVE_TO_BINDIR=\"${LITEVO_TOOLS_RELATIVE_TO_BINDIR}\""
    )

    # Workaround: OpenCV's highgui/imgcodecs pull in GDAL which needs curl
    find_package(CURL QUIET)
    if(CURL_FOUND)
        target_link_libraries(litevo_cli PRIVATE CURL::libcurl)
    endif()

    set_project_warnings(litevo_cli)

    # Install the CLI application
    install(TARGETS litevo_cli
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/litevo/tools"
        FILES_MATCHING
        PATTERN "*.py"
        PATTERN "__pycache__" EXCLUDE
    )
endif()

# ── ROS2 node (optional) ────────────────────────────────────────────────────
if(LITEVO_BUILD_ROS2)
    add_executable(litevo_ros_node
        litevo_ros_node.cpp
    )
    target_link_libraries(litevo_ros_node PRIVATE litevo_core)
    ament_target_dependencies(litevo_ros_node
        rclcpp
        cv_bridge
        tf2_ros
        sensor_msgs
        geometry_msgs
        visualization_msgs
    )
    target_compile_definitions(litevo_ros_node PRIVATE LITEVO_HAS_ROS2)
    set_project_warnings(litevo_ros_node)

    install(TARGETS litevo_ros_node
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
