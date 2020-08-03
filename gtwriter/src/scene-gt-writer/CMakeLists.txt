set(SOURCE_FILES main.cpp 
                 scene.cpp
				 sequence_composer.cpp
				 output_writer.cpp
				 model_renderer.cpp)
	
add_executable(scene-gt-writer ${SOURCE_FILES})
	
target_include_directories(scene-gt-writer PUBLIC
     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../include>)
	  
SET(LIBRARIES_TO_LINK  ${OpenCV_LIBS}
						   ${RENDERER_LIBS}
						   ${Boost_LIBRARIES}
						   ${YAML_CPP_LIBRARIES}
						   nlohmann_json::nlohmann_json)
	
target_link_libraries(scene-gt-writer ${LIBRARIES_TO_LINK})
	
set_target_properties(scene-gt-writer PROPERTIES DEBUG_POSTFIX d)
	
file(GLOB RENDERER_DLLS ${PROJECT_SOURCE_DIR}/../third-party/librenderer/bin/*.dll)
file(COPY ${RENDERER_DLLS} DESTINATION ${CMAKE_INSTALL_PREFIX})
