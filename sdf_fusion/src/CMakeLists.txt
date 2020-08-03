CMAKE_MINIMUM_REQUIRED(VERSION 3.10 FATAL_ERROR)
PROJECT(sdf_fusion)
MESSAGE("Building ${PROJECT_NAME}")

set(CMAKE_CXX_STANDARD 17)

# Install destination
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../dist" CACHE PATH "Install prefix" FORCE )
ENDIF()

include(GNUInstallDirs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

SET(EIGEN_DIR "$ENV{EIGEN_DIR}" CACHE PATH "Path to eigein3.")
SET(BOOST_ROOT "$ENV{BOOST_ROOT}" CACHE PATH "Path to boost.")
SET(OPENCV_DIR "$ENV{OPENCV_DIR}" CACHE PATH "Path to opencv.")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


if((EIGEN_DIR STREQUAL "") OR (BOOST_ROOT STREQUAL "") OR (OPENCV_DIR STREQUAL "") OR (OPENNI_DIR STREQUAL ""))
    MESSAGE(FATAL_ERROR "The ENV variable EIGEN_DIR or BOOST_ROOT or OPENCV_DIR or OPENNI_DIR or REALSENSE_DIR! Please specify path manualy.")
else()
    #SETUP of the lib dependencies
    INCLUDE(cmake/LibPaths.cmake)
    #SETUP of the include paths
    INCLUDE(cmake/IncludePaths.cmake)
endif()

add_subdirectory(aruco_sdffusion)
