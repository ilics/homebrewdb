project(app VERSION 0.1 LANGUAGES CXX)

SET(RGBD_CAPTURE_LIBS_DIR ${PROJECT_SOURCE_DIR}/librgbdcapture/lib CACHE FILEPATH "Capturing library")
LINK_DIRECTORIES(${RGBD_CAPTURE_LIBS_DIR})
MESSAGE(STATUS "RGBD CAPTURE libs dir ${RGBD_CAPTURE_LIBS_DIR}")

set(RGBD_CAPTURE_LIBS optimized librgbdcapture debug librgbdcaptured)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    SET( CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/../../dist" CACHE PATH "Install prefix" FORCE )
ENDIF()

include(GNUInstallDirs)

add_executable(app
   src/main.cpp)
 
target_link_libraries(app ${OpenCV_LIBS}
					  ${RGBD_CAPTURE_LIBS})

set_target_properties(app PROPERTIES DEBUG_POSTFIX d)

target_include_directories(app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
					  
install(TARGETS app EXPORT MyLibraryConfig
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

file(GLOB RGBD_CAPTURE_DLLS ${PROJECT_SOURCE_DIR}/librgbdcapture/bin/*.dll)
file(COPY ${RGBD_CAPTURE_DLLS} DESTINATION ${CMAKE_INSTALL_PREFIX})