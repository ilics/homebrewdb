//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef REFINER_OCCLUSION_HANDLER_HPP
#define REFINER_OCCLUSION_HANDLER_HPP

#include "rgbd_types.hpp"
#include <Eigen/Dense>
#include "configuration.h"
#include <opencv2/core/core.hpp>


class OcclusionHandler {

public:

  OcclusionHandler(Configuration configuration,
      std::vector<RgbdFile> rgbd_image_files, std::vector<Eigen::Matrix4d> camera_poses);

  std::vector<size_t> get_frame_ids_with_visible_model(int model_id, const Eigen::Matrix4d &model_pose);

private:

  bool is_object_visible(const cv::Mat& gt_depth, const cv::Mat& rendered_depth);

  Configuration configuration;
  std::vector<RgbdFile> rgbd_image_files;
  std::vector<Eigen::Matrix4d> camera_poses;

  float object_visibility_threshold;
  float occlusion_threshold;

};

#endif //REFINER_OCCLUSION_HANDLER_HPP
