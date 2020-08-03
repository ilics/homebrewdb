cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(gtwriter VERSION 0.1 LANGUAGES CXX)

# Check if the EVN VARIABLES are set. If not set them.
if ((NOT EXISTS "$ENV{OPENCV_DIR}") OR (NOT EXISTS "$ENV{BOOST_DIR}"))
	MESSAGE(FATAL_ERROR "The ENV variable OPENCV_DIR or BOOST_DIR is not set!")
else((NOT EXISTS "$ENV{OPENCV_DIR}") OR (NOT EXISTS "$ENV{BOOST_DIR}"))
    set(BOOST_ROOT $ENV{BOOST_DIR})
	
	set(Boost_USE_STATIC_LIBS        ON) # only find static libs
    set(Boost_USE_MULTITHREADED      ON)
	
	IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
      SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../dist" CACHE PATH "Install prefix" FORCE )
    ENDIF()
    
	
	set(CMAKE_CXX_STANDARD 17)

	find_package(OpenCV REQUIRED)
	find_package(yaml-cpp REQUIRED)

    find_package(Boost COMPONENTS filesystem REQUIRED)
	find_package(nlohmann_json REQUIRED)
	
	include(cmake/IncludePaths.cmake)
	include(cmake/LibPaths.cmake)
	
	add_subdirectory(model-info-writer)
    add_subdirectory(scene-gt-writer)
    

endif((NOT EXISTS "$ENV{OPENCV_DIR}") OR (NOT EXISTS "$ENV{BOOST_DIR}"))



