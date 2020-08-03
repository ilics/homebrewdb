cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(refine-depth VERSION 0.1 LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)

find_package(OPENCV REQUIRED)
SET(Boost_USE_STATIC_LIBS        ON) # only find static libs
SET(Boost_USE_MULTITHREADED      ON)
FIND_PACKAGE(Boost COMPONENTS filesystem system)

SET(EIGEN_DIR "$ENV{Eigen_DIR}" CACHE PATH "Path to eigein3.")

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} )
MESSAGE(STATUS "Boost lib dirs : ${Boost_LIBRARY_DIRS}")

SET(EIGEN_INCLUDE_DIR ${EIGEN_DIR} CACHE FILEPATH "Eigen include dirpath")
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
MESSAGE(STATUS "Eigen include ${EIGEN_INCLUDE_DIR}")

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
MESSAGE(STATUS "Boost include ${Boost_INCLUDE_DIRS}")

add_executable(refine-depth
    src/main.cpp
    src/marker_detector.cpp)

target_include_directories(refine-depth PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(refine-depth
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES})