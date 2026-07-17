# ── SLAMForge unit tests ───────────────────────────────────────────────────────

set(UNIT_TEST_SOURCES
    test_core_types.cpp
    test_camera.cpp
    test_se3.cpp
    test_sim3.cpp
    test_triangulation.cpp
    test_config.cpp
    test_feature_grid.cpp
    test_frame.cpp
    test_map.cpp
    test_map_export.cpp
    test_matcher.cpp
    test_tracking_initialization.cpp
    test_keyframe.cpp
    test_ba.cpp
    test_local_mapper.cpp
    test_dense_mapper.cpp
    test_loop_closing.cpp
)

foreach(test_src ${UNIT_TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE slamforge_core gtest_main gmock_main)
    set_project_warnings(${test_name})
    add_test(NAME ${test_name} COMMAND ${test_name}
             WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endforeach()
