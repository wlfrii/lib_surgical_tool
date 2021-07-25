# ----------------------------------------------------------------------------
#                               Settings
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.10)

project(lib_instrument VERSION 1.0.0 LANGUAGES CXX)
set(BINARY ${CMAKE_PROJECT_NAME})
message(STATUS "Project: ${BINARY}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

set(MODULE_ROOT ${CMAKE_CURRENT_BINARY_DIR})
message("  Module root: ${MODULE_ROOT}")
set(REPO_ROOT ${MODULE_ROOT}/../../../)
message("  Repo root: ${REPO_ROOT}")
# set(REPO_INC ${REPO_ROOT}/inc)
# if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(CMAKE_INSTALL_PREFIX ${REPO_ROOT}/buildapp/ CACHE PATH "repo root" FORCE)
# endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
message("  CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

# ----------------------------------------------------------------------------
#                     Create target and set properties
# ----------------------------------------------------------------------------

# file(GLOB_RECURSE SRCS LIST_DIRECTORIES true *.h *.cpp)
file(GLOB_RECURSE SRCS *.cpp)
# message("Files to be compiled: ${SRCS}")

add_library(${BINARY} STATIC 
    ${SRCS}
)

# Specifies include directories to use when compiling a given target. The target
# name must have been created by a command such as add_excutable() or add_library().
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
    message("  Found Eigen: ${EIGEN3_INCLUDE_DIRS}")
else(${Eigen3_FOUND})
    message(ERROR "Cannot find Eigen")
endif(${Eigen3_FOUND})
target_include_directories(${BINARY}
    PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/configspcs
        ${CMAKE_CURRENT_BINARY_DIR}/define
)

target_compile_features(${BINARY} PRIVATE cxx_std_14)
target_compile_options(${BINARY} PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -fPIC>)
target_compile_options(${BINARY} PRIVATE -Wall -Werror)

# ----------------------------------------------------------------------------
#                               Installation
# ----------------------------------------------------------------------------

# Define GNU standard installation directories
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${BINARY})
message("  CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message("  INSTALL_CONFIGDIR: ${INSTALL_CONFIGDIR}")

# Set installation path 
install(TARGETS ${BINARY} 
    EXPORT ${BINARY}-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/lib
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_LIBDIR}/include
)

# Bellow is required so that the exported target has the name ${BINARY}
set_target_properties(${BINARY} PROPERTIES
    EXPORT_NAME ${BINARY}
)

# Export the targets to script. i.e.,
# write ${BINARY}-targets information into ${BINARY}-targets.cmake
install(EXPORT ${BINARY}-targets
    FILE 
        ${BINARY}Targets.cmake
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

# Create a library that can be found be find_package()
# i.e., creat a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install the Config, ConfigVersion and Custom find moduls
install(
    FILES 
        ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}ConfigVersion.cmake 
    DESTINATION 
        ${INSTALL_CONFIGDIR}
)

# ----------------------------------------------------------------------------
#                            Export and Register
# ----------------------------------------------------------------------------
export(EXPORT ${BINARY}-targets
    FILE 
        ${CMAKE_CURRENT_BINARY_DIR}/${BINARY}Targets.cmake
)

export(PACKAGE ${BINARY})