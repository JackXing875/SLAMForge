# ── LiteVO tests ────────────────────────────────────────────────────────────

add_subdirectory(unit)

# Keep regression coverage for the Python trajectory utilities in the same
# ctest invocation as the C++ suite.  The test itself adjusts sys.path to use
# the repository's tools directory.  Keep Python/NumPy optional so a C++-only
# test build remains usable on minimal development machines.
if(LITEVO_BUILD_PYTHON_TESTS)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        execute_process(
            COMMAND "${Python3_EXECUTABLE}" -c "import numpy"
            RESULT_VARIABLE LITEVO_NUMPY_IMPORT_RESULT
            OUTPUT_QUIET
            ERROR_QUIET
        )
        if(LITEVO_NUMPY_IMPORT_RESULT EQUAL 0)
            add_test(
                NAME test_trajectory_io
                COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tests/python/test_trajectory_io.py"
            )
            set_tests_properties(test_trajectory_io PROPERTIES
                WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
            )
        else()
            message(WARNING "Python regression tests disabled: NumPy is not importable")
        endif()
    else()
        message(WARNING "Python regression tests disabled: Python3 interpreter not found")
    endif()
endif()

if(TARGET litevo_cli)
    # A child Python tool error must propagate through `litevo_cli eval` as a
    # non-zero process status; otherwise shell scripts and CI report a false
    # successful evaluation.
    add_test(
        NAME test_cli_eval_failure_propagates
        COMMAND $<TARGET_FILE:litevo_cli> eval
            --estimated "${PROJECT_SOURCE_DIR}/tests/data/zero_variance_estimated.tum"
            --groundtruth "${PROJECT_SOURCE_DIR}/tests/data/zero_variance_groundtruth.tum"
    )
    set_tests_properties(test_cli_eval_failure_propagates PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        WILL_FAIL TRUE
    )

    # An image extension alone is not evidence that OpenCV could decode a
    # frame.  A completely unreadable input set must return failure.
    add_test(
        NAME test_cli_rejects_unreadable_images
        COMMAND $<TARGET_FILE:litevo_cli> run
            --config "${PROJECT_SOURCE_DIR}/config/euroc.yaml"
            --input "${PROJECT_SOURCE_DIR}/tests/data/invalid_images"
            --output "${CMAKE_CURRENT_BINARY_DIR}/invalid_images_trajectory.txt"
    )
    set_tests_properties(test_cli_rejects_unreadable_images PROPERTIES
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        WILL_FAIL TRUE
    )
endif()
