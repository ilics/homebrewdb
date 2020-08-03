//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef GT_WRITER_SCENE_H
#define GT_WRITER_SCENE_H

#include <iostream>
#include "model.h"
#include <Eigen/Core>
#include "model_renderer.h"
#include "configuration.h"
#include "frame_model.h"
#include "frame.h"

class Scene {
		
public:

	Scene(const Configuration &p_configuration, const std::string &p_scene_dir, 
		const std::vector<Model> &p_models, const std::vector<Eigen::Matrix4f> &p_camera_poses);

	std::vector<Frame> convert_to_scene_frames();
	size_t get_number_of_frames();


private:
	Configuration configuration;

	std::vector<Model> models;
	std::vector<ModelRenderer> scaled_renderers;
	
	std::vector<Eigen::Matrix4f> frame_poses;
	std::string images_path;
	std::string scene_dir;

};

#endif