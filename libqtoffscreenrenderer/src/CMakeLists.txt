cmake_minimum_required(VERSION 3.7)

project(renderer C CXX)
set(CMAKE_CXX_STANDARD 17)
set(BINARY_NAME librenderer)
include_directories($ENV{EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES painter.cpp model.cpp renderer.cpp) 

find_package(OpenCV REQUIRED)
find_package(OpenGL)

find_package(Qt5OpenGL)

#add_executable(${BINARY_NAME} ${SOURCE_FILES})

add_library(${BINARY_NAME} SHARED ${SOURCE_FILES})
add_library(${BINARY_NAME}_static STATIC ${SOURCE_FILES})

target_include_directories(${BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

target_include_directories(${BINARY_NAME}_static PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)

qt5_use_modules(${BINARY_NAME} Widgets OpenGL)
qt5_use_modules(${BINARY_NAME}_static Widgets OpenGL)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})


target_link_libraries(${BINARY_NAME} ${OpenCV_LIBS}
                               ${Boost_LIBRARIES}
							   ${QT_LIBRARIES}
							   ${OPENGL_LIBRARIES})

target_link_libraries(${BINARY_NAME}_static ${OpenCV_LIBS}
                             ${Boost_LIBRARIES}
							  ${QT_LIBRARIES}
    						  ${OPENGL_LIBRARIES})

set_target_properties(${BINARY_NAME} PROPERTIES DEBUG_POSTFIX d)
set_target_properties(${BINARY_NAME}_static PROPERTIES DEBUG_POSTFIX d)