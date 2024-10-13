message(STATUS "dqpose_BUILD_TESTS is set to ${dqpose_BUILD_TESTS}")

if(dqpose_BUILD_TESTS)
    # Compile dq1 examples
    set(TESTS_NAMES
        test1
    )

    foreach(TEST ${TESTS_NAMES})
        add_executable(${TEST} ${TEST}.cpp)
    endforeach()
endif()