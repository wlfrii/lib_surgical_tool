cmake_minimum_required(VERSION 3.10)
project(lib_instrument_test)

set(CMAKE_CXX_STANDARD 14)

find_package(GTest REQUIRED)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../buildtarget/cmake/)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(lib_instrument REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_SRC)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/inc INC_SRC)
add_executable(${PROJECT_NAME}
    main.cpp
    ${SRC_SRC}
    ${INC_SRC}
)


target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${lib_instrument_LIBRARIES}
    ${GTEST_LIBRARIES}
)
