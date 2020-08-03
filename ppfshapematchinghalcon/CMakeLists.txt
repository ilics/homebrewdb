# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(shape_matching VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package( OPENCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


SET(EIGEN_DIR "$ENV{EIGEN_DIR}" CACHE PATH "Path to eigen3.")

SET(EIGEN_INCLUDE_DIR ${EIGEN_DIR} CACHE FILEPATH "Eigen include dirpath")
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
MESSAGE(STATUS "Eigen include ${EIGEN_INCLUDE_DIR}")

SET(OPENCV_INCLUDE_DIR $ENV{OPENCV_DIR}/include CACHE FILEPATH "OpenCV include dir path")
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
MESSAGE(STATUS "OpenCV include ${OPENCV_INCLUDE_DIR}")

SET(HALCON_INCLUDE_DIR $ENV{HALCONROOT}/include CACHE FILEPATH "HALCON include dir path")
INCLUDE_DIRECTORIES(${HALCON_INCLUDE_DIR})
MESSAGE(STATUS "HALCON include ${HALCON_INCLUDE_DIR}")

SET(HALCON_LIBS_DIR $ENV{HALCONROOT}/lib/x64-win64 CACHE FILEPATH "HALCON lib dir path")
LINK_DIRECTORIES(${HALCON_LIBS_DIR})
MESSAGE(STATUS "HALCON libs dir ${HALCON_LIBS_DIR}")

SET(HALCON_LIBS halcon halconc halconcpp halconcppxl halconcxl halconxl hdevenginecpp hdevenginecppxl libiomp5md)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../../dist" CACHE PATH "Install prefix" FORCE )
ENDIF()

include(GNUInstallDirs)

add_executable(shape_matching
  src/model_matching.cpp)
   
target_link_libraries(shape_matching ${OpenCV_LIBS}
                                    ${HALCON_LIBS})
set_target_properties(shape_matching PROPERTIES DEBUG_POSTFIX d)

target_include_directories(shape_matching PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
					  
install(TARGETS shape_matching EXPORT MyLibraryConfig
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)
