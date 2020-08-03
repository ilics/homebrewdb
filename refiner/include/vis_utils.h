//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef EDGE_ALIGNMENT_VIS_UTILS_H
#define EDGE_ALIGNMENT_VIS_UTILS_H

#include <opencv/cv.hpp>
#include <Eigen/Core>
#include <iostream>
#include <iomanip>
#include <string>
#include <frame_payload.h>

void save_edge_visualization(int height, int width, const std::string& edge_match_out_path, int num_iter,
	const FramePayload& payload, const std::vector<Eigen::Vector4f>& depth_edges, const std::vector<Eigen::Vector4f>& rgb_edges,
	const std::string& in_img_filename);

cv::Mat get_edge_visualization(int height, int width, const FramePayload& payload, const std::vector<Eigen::Vector4f>& depth_edges, const std::vector<Eigen::Vector4f>& rgb_edges);


#endif //EDGE_ALIGNMENT_VIS_UTILS_H
