//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#include "rendering_helper.h"
#include "util.h"
#include <opencv2/core/eigen.hpp>
#include <iomanip>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

RenderingHelper::RenderingHelper(const Configuration &configuration, int model_id, int width, int height) {
  initialize_depth_renderer(configuration, model_id, width, height);
}

string get_model_file(const string &reference_models_dir, int model_id) {
  std::stringstream filename_ss;
  filename_ss << "obj_" << std::setfill('0') << std::setw(6) << model_id << ".ply";
  const std::string filename = filename_ss.str();

  return (fs::path(reference_models_dir) / filename).string();
}

void RenderingHelper::initialize_depth_renderer(const Configuration &configuration, int model_id, int width, int height) {
  RendererConfiguration renderer_configuration;

  renderer_configuration.width = width;
  renderer_configuration.height = height;

  renderer_configuration.model_file = get_model_file(configuration.get_reference_models_dir(), model_id);

  renderer_configuration.intrinsics = configuration.get_intrinsics();

  renderer_configuration.z_near = 0.001f;
  renderer_configuration.z_far = 4.05f;
  renderer = std::shared_ptr<RendererInterface>(get_depth_renderer(renderer_configuration));
}

void RenderingHelper::render(const Eigen::Matrix4d &world_to_camera, cv::Mat &depth, cv::Mat &color) {
  Eigen::Matrix4f world_to_camera_f = world_to_camera.cast<float>();
  renderer->render(world_to_camera_f, depth, color);
}