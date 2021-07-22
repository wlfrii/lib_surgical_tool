cmake_minimum_required(VERSION 3.0)
project(lib_instrument_test)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_subdirectory(inc/gtest)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/inc INC_SRC)
add_executable(${PROJECT_NAME}
    main.cpp
    ${SRC_SRC}
    ${INC_SRC}
)

add_test(
    NAME ${PROJECT_NAME}
    COMMAND ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtest
)