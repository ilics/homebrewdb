//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef H_REFINER_INPUT_HANDLER
#define H_REFINER_INPUT_HANDLER

#include "rgbd_types.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <map>

bool get_rgbd_files(const std::string& scene_dir, std::vector<RgbdFile>& rgbd_files);

bool read_scene_camera_poses(const std::string& scene_dir, std::vector<Eigen::Matrix4d>& camera_poses);

bool get_model_poses(const std::string& scene_dir, const std::string &model_ids_string, std::map<int, Eigen::Matrix4d> &model_poses);

#endif