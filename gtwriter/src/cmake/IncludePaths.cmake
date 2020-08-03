# Set include paths from environment variables
include_directories(${Boost_INCLUDE_DIRS})
	
include_directories(${YAML_CPP_INCLUDE_DIR})

SET(OPENCV_INCLUDE_DIR $ENV{OPENCV_DIR}/include CACHE FILEPATH "OpenCV include dir path")
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
MESSAGE(STATUS "OpenCV include ${OPENCV_INCLUDE_DIR}")

SET(EIGEN_INCLUDE_DIR $ENV{EIGEN_DIR} CACHE FILEPATH "Eigen include dir path")
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
MESSAGE(STATUS "Eigen include ${EIGEN_INCLUDE_DIR}")