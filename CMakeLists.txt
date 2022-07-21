cmake_minimum_required(VERSION 3.10)

project(lib_surgical_tool
    VERSION 1.1.0
    DESCRIPTION "A library for continuum surgical tool"
    LANGUAGES CXX)
message("==== Build Library: ${PROJECT_NAME} ====")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------------------
#                   Create target and set properties
# --------------------------------------------------------------------
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} STATIC 
    ${SRCS}
)

# We need to tell CMake that we want to use different include 
# directories depending on if we are building the library or using it 
# from an installed location. If we don't do this, when CMake creates 
# the export information it will export a path that is specific to the 
# current build directory and will not be valid for other projects.
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
        $<BUILD_INTERFACE:${lib_math_INCLUDE_DIRS}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        ${lib_math_LIBRARIES}
)


file(GLOB EXPORT_H include/*.h)
file(COPY ${EXPORT_H}
    DESTINATION ${CMAKE_BINARY_DIR}/include/
)
file(GLOB EXPORT_H include/define/*.h)
file(COPY ${EXPORT_H}
    DESTINATION ${CMAKE_BINARY_DIR}/include/define
)

# Build the test
option(BUILD_TEST "select to build the test case" ON)
if(BUILD_TEST)
    add_subdirectory(test)
endif()
