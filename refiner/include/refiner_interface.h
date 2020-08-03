//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef REFINER_INTERFACE_H
#define REFINER_INTERFACE_H

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include "rgbd_types.hpp"

//#define RefinerLib_EXPORTS
//
//#ifdef RefinerLib_EXPORTS
//#define RefinerAPI __declspec(dllexport)
//#else
//#define RefinerAPI __declspec(dllimport)
//#endif

//struct RefinementInput
//{
//  Eigen::Matrix4d model_pose;
//  std::vector<std::string> image_files;
//  std::vector<Eigen::Matrix4d> sequense_poses;
//};

struct RefinementInput
{
  std::map<int, Eigen::Matrix4d> model_poses;
  std::vector<RgbdFile> rgbd_image_files;
  std::vector<Eigen::Matrix4d> camera_poses;
};



class RefinerInterface {

public:
	virtual Eigen::Matrix4d run_refinement(RefinementInput &input) = 0;
};

typedef RefinerInterface* RefinerInterfaceHandle;

//#ifdef __cplusplus
//#   define EXTERN_C     extern "C"
//#else
//#   define EXTERN_C
//#endif // __cplusplus

//EXTERN_C
//RefinerAPI RefinerInterfaceHandle get_refiner(std::string config_file);

RefinerInterfaceHandle get_refiner(const std::string& config_file);
#endif