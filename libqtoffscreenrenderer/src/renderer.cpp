//######################################################################
//#   Liboffscreenrenderer Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "renderer.h"

Renderer::Renderer(RendererConfiguration& configuration) :
  intrinsics(configuration.intrinsics), z_near(configuration.z_near), z_far(configuration.z_far)
{	
	Painter::z_near = configuration.z_near;
	Painter::z_far= configuration.z_far;
	Painter::width = configuration.width;
	Painter::height = configuration.height;

	model.loadPLY(configuration.model_file);
}

void Renderer::render(Eigen::Matrix4f &pose_matrix, cv::Mat &depth, cv::Mat &color)
{
	Eigen::Isometry3f pose;
	pose.setIdentity();

	pose.linear() = pose_matrix.block<3, 3>(0, 0);
	pose.translation() = pose_matrix.block<3, 1>(0, 3);

	RealWorldCamera cam(intrinsics, pose, painter.getNear(), painter.getFar());

	int x = 0, y = 0, w = 0, h = 0;

	painter.clearObjects();
	painter.setBackground(0, 0, 0);
	painter.addPaintObject(&cam);
	painter.addPaintObject(&model);
	painter.paint(x, y, w, h);
	painter.copyDepthTo(depth);
	painter.copyColorTo(color);
}

#ifdef _WIN32
RendererAPI
#endif
RendererInterfaceHandle get_depth_renderer(RendererConfiguration &configuration)
{
	return new Renderer(configuration);
}