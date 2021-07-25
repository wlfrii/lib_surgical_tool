cmake_minimum_required(VERSION 3.10)
project(lib_instrument_test)

set(CMAKE_CXX_STANDARD 14)

find_package(GTest REQUIRED)
find_package(lib_instrument REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${lib_instrument_INCLUDE_DIR}
)

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

link_directories(${CMAKE_BINARY_DIR}/src)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${GTEST_LIBRARIES}
    ${lib_instrument_LIBRARIES}
)
