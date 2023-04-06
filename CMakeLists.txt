cmake_minimum_required(VERSION 3.10)

project(lib_surgical_tool
    VERSION 1.1.0
    DESCRIPTION "A library for continuum surgical tool"
    LANGUAGES CXX)
message("==== Build Library: ${PROJECT_NAME} ====")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------

# Add lib_math (within Eigen as denpendency)
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../buildtarget/)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
find_package(lib_math REQUIRED)
if(${lib_math_FOUND})
    message(STATUS "lib_math found")
    message(STATUS "  include path: ${lib_math_INCLUDE_DIRS}")
else()
    message(ERROR "Cannot find lib_math")
endif()

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} STATIC 
    ${SRCS}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${lib_math_LIBRARIES}
)

# file(GLOB EXPORT_H include/*.h)
# file(COPY ${EXPORT_H}
#     DESTINATION ${CMAKE_BINARY_DIR}/include/
# )
# file(GLOB EXPORT_H include/define/*.h)
# file(COPY ${EXPORT_H}
#     DESTINATION ${CMAKE_BINARY_DIR}/include/define
# )

# Build the test
option(BUILD_TEST "select to build the test case" ON)
if(BUILD_TEST)
    add_subdirectory(test)
endif()
