# ── LiteVO CLI application ──────────────────────────────────────────────────

add_executable(litevo_cli
    litevo_cli.cpp
)

target_link_libraries(litevo_cli PRIVATE
    litevo_core
    CLI11::CLI11
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

# ── ROS2 node (optional) ────────────────────────────────────────────────────
if(LITEVO_BUILD_ROS2)
    add_executable(litevo_ros_node
        litevo_ros_node.cpp
    )
    target_link_libraries(litevo_ros_node PRIVATE
        litevo_core
        ${rclcpp_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
    )
    target_include_directories(litevo_ros_node PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
    )
    target_compile_definitions(litevo_ros_node PRIVATE LITEVO_HAS_ROS2)
    set_project_warnings(litevo_ros_node)

    install(TARGETS litevo_ros_node
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()
