set(TEST_NAME "${PROJECT_NAME}_run")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_run.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)


set(TEST_NAME "${PROJECT_NAME}_ik")
message("==== Build Test: ${TEST_NAME} ====")

add_executable(${TEST_NAME}
    test_ik.cpp
)
target_link_libraries(${TEST_NAME}
    ${PROJECT_NAME}
)


set(TEST_NAME "${PROJECT_NAME}_test")
message("==== Build Test: ${TEST_NAME} ====")

# file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(${TEST_NAME}
    test_surgical_tool.cpp
)
set(CATCH_INCLUDE_DIR /usr/local/Cellar/catch2/2.13.9/include/)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CATCH_INCLUDE_DIR}>
)
target_link_libraries(${TEST_NAME}
    PUBLIC
        ${PROJECT_NAME}
)
