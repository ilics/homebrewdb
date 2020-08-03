# Define library. Only source files here!
project(librgbdcapture VERSION 0.1 LANGUAGES CXX)


set(SOURCE_FILES src/openni_camera.cpp
			 src/rgbd_camera.cpp
			 src/realsense_camera.cpp
			 src/kinectv2_camera.cpp
			 src/kinect_azure_camera.cpp)


add_library(librgbdcapture SHARED ${SOURCE_FILES})

add_library(librgbdcapture_static STATIC ${SOURCE_FILES})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.

target_include_directories(librgbdcapture PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include/>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

target_include_directories(librgbdcapture_static PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include/>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Depend on a library that we defined in the top-level file
target_link_libraries(librgbdcapture ${OpenCV_LIBS}
					  ${OPENNI2_LIBS}
					  ${REALSENSE_LIBS}
					  ${KINECT2_LIBS}
					  ${KINECT_AZURE_LIBS})

target_link_libraries(librgbdcapture_static ${OpenCV_LIBS}
					  ${OPNNI2_LIBS}
					  ${REALSENSE_LIBS}
					  ${KINECT2_LIBS}
					  ${KINECT_AZURE_LIBS})

set_target_properties(librgbdcapture PROPERTIES DEBUG_POSTFIX d)
set_target_properties(librgbdcapture_static PROPERTIES DEBUG_POSTFIX d)

# 'make install' to the correct location
install(TARGETS librgbdcapture EXPORT MyLibraryConfig
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

install(TARGETS librgbdcapture_static EXPORT MyLibraryConfig
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)  # This is for Windows
install(DIRECTORY include/ DESTINATION include)

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into <prefix>/cmake.
install(EXPORT MyLibraryConfig DESTINATION ${CMAKE_INSTALL_PREFIX})


# This makes the project importable from the build directory
export(TARGETS librgbdcapture FILE MyLibraryConfig.cmake)
export(TARGETS librgbdcapture_static FILE MyLibraryConfig.cmake)
