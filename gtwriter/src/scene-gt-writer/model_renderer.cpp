//######################################################################
//#   GT Writter Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "model_renderer.h"


ModelRenderer::ModelRenderer(const Model &model, const Configuration &configuration, bool scale_focal_length)
{
	RendererConfiguration renderer_config;

	Eigen::Matrix3f intrinsics = configuration.get_intrinsics();

	if (scale_focal_length)
	{
		float scaling_factor = configuration.get_focal_length_scale();
		intrinsics(0, 0) = scaling_factor * intrinsics(0, 0);
		intrinsics(1, 1) = scaling_factor * intrinsics(1, 1);
	}

	renderer_config.intrinsics = intrinsics;

	renderer_config.height = configuration.get_image_height();
	renderer_config.width = configuration.get_image_width();
	renderer_config.z_near = 0.001f;
	renderer_config.z_far = 4.05f;
	renderer_config.model_file = model.model_file;

	renderer = std::shared_ptr<RendererInterface>(get_depth_renderer(renderer_config));
}

void ModelRenderer::render(Eigen::Matrix4f &world_to_cam, cv::Mat &depth, cv::Mat &color)
{
	renderer->render(world_to_cam, depth, color);
}
