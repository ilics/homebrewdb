link_directories(${Boost_LIBRARY_DIRS} ) 
	
SET(OPENCV_LIBS opencv_core opencv_imgcodecs opencv_calib3d opencv_highgui opencv_imgproc opencv_features2d opencv_flann)

SET(RENDERER_LIBS_DIR ${PROJECT_SOURCE_DIR}/../third-party/librenderer/lib CACHE FILEPATH "renderer lib dir path")
LINK_DIRECTORIES(${RENDERER_LIBS_DIR})

SET(RENDERER_LIBS optimized librenderer debug librendererd)