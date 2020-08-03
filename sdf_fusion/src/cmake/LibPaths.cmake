FIND_PACKAGE(OpenCV)
MESSAGE(STATUS "OpenCV libs: ${OpenCV_LIBS}")

SET(Boost_USE_STATIC_LIBS        ON) # only find static libs
SET(Boost_USE_MULTITHREADED      ON)
FIND_PACKAGE(Boost COMPONENTS filesystem)
 
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS} ) 
MESSAGE(STATUS "Boost lib dirs : ${Boost_LIBRARY_DIRS}")