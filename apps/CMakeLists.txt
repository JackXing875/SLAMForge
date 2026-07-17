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
    if(UNIX AND NOT APPLE AND SLAMFORGE_ENABLE_DENSE)
        set_target_properties(slamforge_cli PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
    endif()

    # Install the CLI application
    install(TARGETS slamforge_cli
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Desktop
    )
    install(DIRECTORY "${PROJECT_SOURCE_DIR}/tools/"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/slamforge/tools"
        COMPONENT Desktop
        FILES_MATCHING
        PATTERN "*.py"
        PATTERN "__pycache__" EXCLUDE
    )
endif()

# ── SLAMForge Desktop (Qt 6) ────────────────────────────────────────────────
if(SLAMFORGE_BUILD_DESKTOP)
    add_executable(slamforge_desktop WIN32 MACOSX_BUNDLE
        desktop/main.cpp
        desktop/map_view.cpp
        desktop/map_view.h
        desktop/main_window.cpp
        desktop/main_window.h
    )
    set_source_files_properties(
        "${PROJECT_SOURCE_DIR}/packaging/linux/org.slamforge.SLAMForge.svg"
        PROPERTIES QT_RESOURCE_ALIAS "app-icon.svg"
    )
    qt_add_resources(slamforge_desktop "slamforge_desktop_assets"
        PREFIX "/slamforge"
        FILES "${PROJECT_SOURCE_DIR}/packaging/linux/org.slamforge.SLAMForge.svg"
    )

    target_link_libraries(slamforge_desktop PRIVATE Qt6::Widgets)
    target_compile_definitions(slamforge_desktop PRIVATE
        "SLAMFORGE_DESKTOP_VERSION=\"${SLAMFORGE_VERSION_STRING}\""
    )
    set_project_warnings(slamforge_desktop)
    add_dependencies(slamforge_desktop slamforge_cli)

    # Keep a source-tree desktop build runnable without an install step when
    # the caller supplied the same pinned model used by release packaging.
    if(SLAMFORGE_DEPTH_MODEL_FILE AND EXISTS "${SLAMFORGE_DEPTH_MODEL_FILE}")
        add_custom_command(TARGET slamforge_desktop POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:slamforge_desktop>/models"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SLAMFORGE_DEPTH_MODEL_FILE}"
                "$<TARGET_FILE_DIR:slamforge_desktop>/models/depth_anything_v2_vits_dynamic.onnx"
            VERBATIM
        )
    endif()

    if(WIN32)
        set_target_properties(slamforge_desktop PROPERTIES
            OUTPUT_NAME "SLAMForge Desktop"
        )
    endif()

    install(TARGETS slamforge_desktop
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Desktop
    )
    install(FILES "${PROJECT_SOURCE_DIR}/docs/desktop.md"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/slamforge"
        COMPONENT Desktop
    )
    if(UNIX AND NOT APPLE)
        install(FILES "${PROJECT_SOURCE_DIR}/packaging/linux/org.slamforge.SLAMForge.desktop"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/applications"
            COMPONENT Desktop
        )
        install(FILES "${PROJECT_SOURCE_DIR}/packaging/linux/org.slamforge.SLAMForge.svg"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps"
            COMPONENT Desktop
        )
        install(FILES "${PROJECT_SOURCE_DIR}/packaging/linux/org.slamforge.SLAMForge.appdata.xml"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/metainfo"
            COMPONENT Desktop
        )
    endif()

    if(SLAMFORGE_BUILD_TESTS)
        add_test(NAME test_desktop_result_view
            COMMAND slamforge_desktop
                --smoke-result "${PROJECT_SOURCE_DIR}/tests/data/desktop-result"
        )
        if(UNIX AND NOT APPLE)
            set_tests_properties(test_desktop_result_view PROPERTIES
                ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
            )
        endif()
    endif()
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
