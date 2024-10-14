message(STATUS "dqpose_BUILD_EXAMPLES is set to ${dqpose_BUILD_EXAMPLES}")

if(dqpose_BUILD_EXAMPLES)
    # Compile dq1 examples
    set(EXAMPLE_NAMES
        example_quat
        example_dualquat
    )

    foreach(EXAMPLE ${EXAMPLE_NAMES})
        add_executable(${EXAMPLE} ${EXAMPLE}.cpp)
    endforeach()
endif()