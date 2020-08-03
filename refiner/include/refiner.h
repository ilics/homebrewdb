//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef REFINER_H
#define REFINER_H

#include <Eigen/Dense>
#include <iostream>
#include <opencv/cv.hpp>
#include "configuration.h"
#include "frame_payload.h"
#include <nlohmann/json.hpp>
#include "refiner_interface.h"
using json = nlohmann::json;

class Refiner
{
public:
	Refiner(const Configuration &p_configuration);
	std::map<int, Eigen::Matrix4d> refine_model_poses(const RefinementInput& input);

private:
  Eigen::Matrix4d refine_model_pose(const std::vector<Eigen::Matrix4d> &camera_poses, const std::vector<cv::Mat> &grayscale_images,
                    const Eigen::Matrix4d &model_pose, int model_id);

  Configuration configuration;
};
#endif