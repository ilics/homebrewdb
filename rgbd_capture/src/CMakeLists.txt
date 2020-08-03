# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(RGBD_Capture VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package( OPENCV REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(OpenMP)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

SET(BOOST_ROOT "$ENV{BOOST_ROOT}" CACHE PATH "Path to boost.")

set(KINECT_AZURE_SDK_DIR $ENV{KINECT_AZURE_SDK_DIR} CACHE FILEPATH "Kinect Azure SDK dir")


# Check if the EVN VARIABLES are set. If not set them.
if ((NOT EXISTS "$ENV{OPENNI2_LIB64}") OR (NOT EXISTS "$ENV{OPENCV_DIR}"))
	MESSAGE(FATAL_ERROR "The ENV variable OPENNI2_DIR or OPENCV_DIR is not set!")
else((NOT EXISTS "$ENV{OPENNI2_LIB64}") OR (NOT EXISTS "$ENV{OPENCV_DIR}"))
	#SETUP of the lib dependencies
	INCLUDE(cmake/LibPaths.cmake)

	#SETUP of the include paths
	INCLUDE(cmake/IncludePaths.cmake)
endif((NOT EXISTS "$ENV{OPENNI2_LIB64}") OR (NOT EXISTS "$ENV{OPENCV_DIR}"))

# Targets that we develop
add_subdirectory(librgbdcapture)
add_subdirectory(app)

