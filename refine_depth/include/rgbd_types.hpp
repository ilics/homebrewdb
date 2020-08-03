//######################################################################
//#   Refine_depth Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef RGBD_TYPES
#define RGBD_TYPES
#include <iostream>
#include <opencv2/core/core.hpp>

typedef std::pair<std::string, std::string> RgbdFile;
typedef std::pair<cv::Mat, cv::Mat> RgbdFrame;

#endif