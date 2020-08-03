set(SOURCE_FILES main.cpp)
	
add_executable(model-info-writer ${SOURCE_FILES})
	
target_include_directories(model-info-writer PUBLIC
     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/../include>)
	  
SET(LIBRARIES_TO_LINK  ${OpenCV_LIBS}
						   ${Boost_LIBRARIES}
						   ${YAML_CPP_LIBRARIES}
						   nlohmann_json::nlohmann_json)
	
target_link_libraries(model-info-writer ${LIBRARIES_TO_LINK})	
set_target_properties(model-info-writer PROPERTIES DEBUG_POSTFIX d)