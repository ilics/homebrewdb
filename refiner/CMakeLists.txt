cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(refiner VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
   SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/dist" CACHE PATH "Install prefix" FORCE )
endif()


set(Boost_USE_STATIC_LIBS ON) # only find static libs
set(Boost_USE_MULTITHREADED ON)

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS filesystem)
find_package(nlohmann_json REQUIRED)

find_package(Ceres REQUIRED)
find_package(Qt5 COMPONENTS Xml Widgets Gui OpenGL REQUIRED)

set(RENDERER_LIBS_DIR ${PROJECT_SOURCE_DIR}/third-party/librenderer/lib CACHE FILEPATH "renderer lib dir path")
link_directories(${RENDERER_LIBS_DIR})
set(RENDERER_LIBS optimized librenderer debug librendererd)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

MESSAGE(STATUS "Renderer libs dir ${RENDERER_LIBS_DIR}")

set(SOURCE_FILES src/main.cpp
		            src/configuration_parser.cpp
					src/rendering_helper.cpp
                    src/correspondence_finder.cpp
					src/refiner.cpp
		            src/occlusion_handler.cpp
					src/vis_utils.cpp
					src/optimizer.cpp
					src/input_handler.cpp)

add_executable(refiner ${SOURCE_FILES})

target_include_directories(refiner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
	
target_include_directories(refiner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

file(GLOB RENDERER_DLLS ${PROJECT_SOURCE_DIR}/third-party/librenderer/bin/*.dll)
file(COPY ${RENDERER_DLLS} DESTINATION ${CMAKE_INSTALL_PREFIX})

SET(LIBRARIES_TO_LINK ${CERES_LIBRARIES} 
		               ${OpenCV_LIBS}
					   ${RENDERER_LIBS}
					   ${Boost_LIBRARIES}
					   Qt5::Widgets
					   Qt5::OpenGL
					   Qt5::Widgets
					   Qt5::Gui
					   Qt5::Xml
					   nlohmann_json::nlohmann_json)

target_link_libraries(refiner ${LIBRARIES_TO_LINK})
set_target_properties(refiner PROPERTIES DEBUG_POSTFIX d)