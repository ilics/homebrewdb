//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef POSEVALIDATOR_HPP
#define POSEVALIDATOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

struct PoseValidationPayload
{
    cv::Mat rgb_image;
	cv::Mat depth_image;
	Eigen::Isometry3f pose;
	Eigen::Matrix3f intrinsics;
	std::vector<float> volume;
};

bool validate_pose(const PoseValidationPayload &payload);

#endif