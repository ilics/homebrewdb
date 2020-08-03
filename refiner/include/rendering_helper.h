//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef RENDERING_HELPER_H
#define RENDERING_HELPER_H
#include "configuration.h"
#include "renderer.h"
#include <Eigen/Dense>

class RenderingHelper
{
  public:
	  RenderingHelper(const Configuration &configuration, int model_id, int width, int height);

	  void render(const Eigen::Matrix4d& world_to_camera, cv::Mat &depth, cv::Mat &color);

  private:
	  void initialize_depth_renderer(const Configuration &configuration, int model_id, int width, int height);
	  std::shared_ptr<RendererInterface> renderer;
};

#endif