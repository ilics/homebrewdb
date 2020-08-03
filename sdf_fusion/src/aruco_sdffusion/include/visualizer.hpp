//######################################################################
//#   SDF_Fusion Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                or under supervision of Slobodan Ilic
//#######################################################################

#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

void visualize_volume(const cv::Mat& input_image, const Eigen::Matrix3f& intrinsics, const Eigen::Isometry3f& pose, const std::vector<float>& volume);



#endif
