//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef FRAME_PAYLOAD_H
#define FRAME_PAYLOAD_H

#include "correspondence_finder.h"
#include <opencv/cv.hpp>
#include <Eigen/Core>

struct FramePayload
{
	Eigen::Matrix4d scene_pose;
	Correspondence correspondence;
};

#endif