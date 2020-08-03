//######################################################################
//#   Refiner Module 
//#   
//#   Copyright (C) 2020 Siemens AG
//#   SPDX-License-Identifier: MIT
//#   Author 2020: This module has been developed by 
//#                Roman Kaskman under supervision of Slobodan Ilic
//#######################################################################

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <iostream>
#include <Eigen/Core>
#include "frame_payload.h"

Eigen::Matrix4d optimize_model_pose(const std::vector<FramePayload> &frame_payloads, const Eigen::Matrix4d &model_pose, const Eigen::Matrix3d &intrinsics, double residuals_std_dev);
#endif
