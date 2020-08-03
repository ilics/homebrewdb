project(aruco_sdffusion VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(aruco_sdffusion
    src/main.cpp
	src/marker_detector.cpp
	src/sdf_fusion.cpp
	src/marching_cubes.cpp
	src/types.cpp
	src/configuration.cpp
	src/configuration_parser.cpp
	src/input_preprocessor.cpp
	src/pose_validator.cpp
	src/visualizer.cpp
	src/reconstructor_3d.cpp)

target_include_directories(aruco_sdffusion PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
	
# Define the libraries this project depends upon
target_link_libraries(aruco_sdffusion
	${OpenCV_LIBS}
	${Boost_LIBRARIES})

set_target_properties(aruco_sdffusion PROPERTIES DEBUG_POSTFIX d)

# Installation

install(TARGETS aruco_sdffusion EXPORT MyLibraryConfig
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)


